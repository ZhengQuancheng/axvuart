//! Virtual PL011 UART device implementation

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use spin::Mutex;

use crate::regs::*;

/// Callback type for interrupt injection
pub type IrqCallback = Box<dyn Fn() + Send + Sync>;

/// Configuration for Virtual PL011 device
#[derive(Debug, Clone)]
pub struct VirtPL011Config {
    pub base_gpa: usize,
    pub length: usize,
    pub irq_id: usize,
}

impl Default for VirtPL011Config {
    fn default() -> Self {
        Self {
            base_gpa: 0x0900_0000,
            length: 0x1000,
            irq_id: 33,
        }
    }
}

/// Internal mutable state of the PL011 UART
struct PL011State {
    // Status & Control Registers
    flags: FlagRegister,
    cr: ControlRegister,
    lcr: LineControlRegister,
    rsr: u32,

    // Baud Rate Registers
    ibrd: u32,
    fbrd: u32,
    ilpr: u32,

    // Interrupt Registers
    int_enabled: InterruptBits,
    int_level: InterruptBits,
    ifls: u32,

    // DMA Control
    dmacr: u32,

    // RX FIFO
    read_fifo: VecDeque<u32>,
    read_trigger: usize,
}

impl Default for PL011State {
    fn default() -> Self {
        Self {
            flags: FlagRegister::TXFE | FlagRegister::RXFE,
            cr: ControlRegister::RXE | ControlRegister::TXE,
            lcr: LineControlRegister::empty(),
            rsr: 0,
            ibrd: 0,
            fbrd: 0,
            ilpr: 0,
            int_enabled: InterruptBits::empty(),
            int_level: InterruptBits::TX,
            ifls: DEFAULT_IFLS,
            dmacr: 0,
            read_fifo: VecDeque::with_capacity(PL011_FIFO_DEPTH),
            read_trigger: 1,
        }
    }
}

impl PL011State {
    fn fifo_depth(&self) -> usize {
        if self.lcr.fifo_enabled() {
            PL011_FIFO_DEPTH
        } else {
            1
        }
    }

    fn can_receive(&self) -> bool {
        self.read_fifo.len() < self.fifo_depth()
    }

    fn set_read_trigger(&mut self) {
        let level = (self.ifls >> 3) & 0x7;
        let depth = self.fifo_depth();

        self.read_trigger = match level {
            0 => depth / 8,
            1 => depth / 4,
            2 => depth / 2,
            3 => (depth * 3) / 4,
            4 => (depth * 7) / 8,
            _ => depth, // Reserved values
        };

        if self.read_trigger == 0 {
            self.read_trigger = 1;
        }
    }
}

/// Virtual PL011 UART Device
pub struct VirtPL011 {
    config: VirtPL011Config,
    state: Mutex<PL011State>,
    irq_callback: Mutex<Option<IrqCallback>>,
}

impl VirtPL011 {
    pub fn new(config: VirtPL011Config) -> Self {
        info!(
            "VirtPL011: created at GPA {:#x}, IRQ {}",
            config.base_gpa, config.irq_id
        );

        Self {
            config,
            state: Mutex::new(PL011State::default()),
            irq_callback: Mutex::new(None),
        }
    }

    pub fn base_gpa(&self) -> usize {
        self.config.base_gpa
    }

    pub fn length(&self) -> usize {
        self.config.length
    }

    pub fn irq_id(&self) -> usize {
        self.config.irq_id
    }

    pub fn set_irq_callback(&self, callback: IrqCallback) {
        *self.irq_callback.lock() = Some(callback);
    }

    pub fn can_receive(&self) -> bool {
        self.state.lock().can_receive()
    }

    /// Push a character into the RX FIFO (Host -> Guest)
    pub fn rx_push(&self, data: u8) -> bool {
        let mut state = self.state.lock();

        let depth = state.fifo_depth();
        if state.read_fifo.len() >= depth {
            trace!("VirtPL011: RX FIFO full, dropping {:#x}", data);
            return false;
        }

        state.read_fifo.push_back(data as u32);

        // Update flags
        state.flags.remove(FlagRegister::RXFE);
        if state.read_fifo.len() >= depth {
            state.flags.insert(FlagRegister::RXFF);
        }

        // Set RX interrupt if above trigger level
        if state.read_fifo.len() >= state.read_trigger {
            state.int_level.insert(InterruptBits::RX);
        }

        drop(state);
        self.update_interrupt();
        true
    }

    fn update_interrupt(&self) {
        let state = self.state.lock();
        let masked = state.int_level & state.int_enabled;
        let should_irq = !masked.is_empty();

        if should_irq {
            drop(state);
            if let Some(ref callback) = *self.irq_callback.lock() {
                callback();
            }
        }
    }

    /// Handle TX write
    fn write_tx_data(&self, data: u8) {
        let mut state = self.state.lock();

        if !state.cr.contains(ControlRegister::UARTEN) {
            warn!("VirtPL011: Write to disabled UART");
        }
        if !state.cr.contains(ControlRegister::TXE) {
            warn!("VirtPL011: Write with TX disabled");
        }

        let loopback = state.cr.contains(ControlRegister::LBE);

        // TX always succeeds immediately, set TX interrupt
        state.int_level.insert(InterruptBits::TX);

        drop(state);

        // Loopback: push to local RX
        if loopback {
            self.rx_push(data);
        }

        // Update interrupt state
        self.update_interrupt();
    }

    /// Handle RX read
    fn read_rx_data(&self) -> u32 {
        let mut state = self.state.lock();

        if state.read_fifo.is_empty() {
            return 0;
        }

        let value = state.read_fifo.pop_front().unwrap_or(0);

        // Update flags
        state.flags.remove(FlagRegister::RXFF);
        if state.read_fifo.is_empty() {
            state.flags.insert(FlagRegister::RXFE);
        }
        if state.read_fifo.len() < state.read_trigger {
            state.int_level.remove(InterruptBits::RX);
        }

        // Update RSR with error flags
        state.rsr = (value >> 8) & 0xF;

        drop(state);
        self.update_interrupt();

        value
    }

    /// Read register
    pub(crate) fn read_reg(&self, offset: usize) -> u32 {
        let state = self.state.lock();

        let value = match offset {
            UART_DR => {
                drop(state);
                return self.read_rx_data();
            }
            UART_RSR_ECR => state.rsr,
            UART_FR => state.flags.bits(),
            UART_ILPR => state.ilpr,
            UART_IBRD => state.ibrd,
            UART_FBRD => state.fbrd,
            UART_LCR_H => state.lcr.bits(),
            UART_CR => state.cr.bits(),
            UART_IFLS => state.ifls,
            UART_IMSC => state.int_enabled.bits(),
            UART_RIS => state.int_level.bits(),
            UART_MIS => (state.int_level & state.int_enabled).bits(),
            UART_DMACR => state.dmacr,

            // Identification registers
            UART_PERIPH_ID0 => PL011_PERIPH_ID[0] as u32,
            UART_PERIPH_ID1 => PL011_PERIPH_ID[1] as u32,
            UART_PERIPH_ID2 => PL011_PERIPH_ID[2] as u32,
            UART_PERIPH_ID3 => PL011_PERIPH_ID[3] as u32,
            UART_PCELL_ID0 => PL011_PCELL_ID[0] as u32,
            UART_PCELL_ID1 => PL011_PCELL_ID[1] as u32,
            UART_PCELL_ID2 => PL011_PCELL_ID[2] as u32,
            UART_PCELL_ID3 => PL011_PCELL_ID[3] as u32,

            _ => {
                warn!("VirtPL011: Read from unknown register {:#x}", offset);
                0
            }
        };

        value
    }

    /// Write register
    pub(crate) fn write_reg(&self, offset: usize, value: u32) {
        match offset {
            UART_DR => {
                self.write_tx_data(value as u8);
            }
            UART_RSR_ECR => {
                self.state.lock().rsr = 0;
            }
            UART_FR => {
                // Read-only
            }
            UART_ILPR => {
                self.state.lock().ilpr = value;
            }
            UART_IBRD => {
                self.state.lock().ibrd = value & IBRD_MASK;
            }
            UART_FBRD => {
                self.state.lock().fbrd = value & FBRD_MASK;
            }
            UART_LCR_H => {
                let mut state = self.state.lock();
                let old_fen = state.lcr.fifo_enabled();
                state.lcr = LineControlRegister::from_bits_truncate(value);
                let new_fen = state.lcr.fifo_enabled();
                // Clear FIFOs on FIFO enable/disable
                if old_fen != new_fen {
                    state.read_fifo.clear();
                    state.flags.insert(FlagRegister::RXFE | FlagRegister::TXFE);
                    state.flags.remove(FlagRegister::RXFF | FlagRegister::TXFF);
                }

                state.set_read_trigger();
            }
            UART_CR => {
                self.state.lock().cr = ControlRegister::from_bits_truncate(value);
            }
            UART_IFLS => {
                let mut state = self.state.lock();
                state.ifls = value;
                state.set_read_trigger();
            }
            UART_IMSC => {
                self.state.lock().int_enabled = InterruptBits::from_bits_truncate(value);
                self.update_interrupt();
            }
            UART_ICR => {
                let clear_bits = InterruptBits::from_bits_truncate(value);
                self.state.lock().int_level.remove(clear_bits);
                self.update_interrupt();
            }
            UART_DMACR => {
                self.state.lock().dmacr = value;
                if value & 0x3 != 0 {
                    warn!("VirtPL011: DMA not supported");
                }
            }
            _ => {
                warn!("VirtPL011: Write to unknown register {:#x}", offset);
            }
        }
    }
}

impl core::fmt::Debug for VirtPL011 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let state = self.state.lock();
        f.debug_struct("VirtPL011")
            .field("base_gpa", &format_args!("{:#x}", self.config.base_gpa))
            .field("irq_id", &self.config.irq_id)
            .field("flags", &state.flags)
            .field("rx_count", &state.read_fifo.len())
            .finish()
    }
}
