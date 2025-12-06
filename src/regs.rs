//! PL011 UART Register Definitions

use bitflags::bitflags;

// ============================================================================
// Register Offsets
// ============================================================================

/// UART Data Register - Read/Write
/// Reading returns received data, writing transmits data
pub const UART_DR: usize = 0x000;

/// UART Receive Status Register / Error Clear Register
pub const UART_RSR_ECR: usize = 0x004;

/// UART Flag Register - Read Only
pub const UART_FR: usize = 0x018;

/// UART IrDA Low-Power Counter Register
pub const UART_ILPR: usize = 0x020;

/// UART Integer Baud Rate Divisor Register
pub const UART_IBRD: usize = 0x024;

/// UART Fractional Baud Rate Divisor Register
pub const UART_FBRD: usize = 0x028;

/// UART Line Control Register
pub const UART_LCR_H: usize = 0x02C;

/// UART Control Register
pub const UART_CR: usize = 0x030;

/// UART Interrupt FIFO Level Select Register
pub const UART_IFLS: usize = 0x034;

/// UART Interrupt Mask Set/Clear Register
pub const UART_IMSC: usize = 0x038;

/// UART Raw Interrupt Status Register
pub const UART_RIS: usize = 0x03C;

/// UART Masked Interrupt Status Register
pub const UART_MIS: usize = 0x040;

/// UART Interrupt Clear Register
pub const UART_ICR: usize = 0x044;

/// UART DMA Control Register
pub const UART_DMACR: usize = 0x048;

// Identification Registers (Read-Only) ???
pub const UART_PERIPH_ID0: usize = 0xFE0;
pub const UART_PERIPH_ID1: usize = 0xFE4;
pub const UART_PERIPH_ID2: usize = 0xFE8;
pub const UART_PERIPH_ID3: usize = 0xFEC;

/// PrimeCell ID registers (4 bytes) ???
pub const UART_PCELL_ID0: usize = 0xFF0;
pub const UART_PCELL_ID1: usize = 0xFF4;
pub const UART_PCELL_ID2: usize = 0xFF8;
pub const UART_PCELL_ID3: usize = 0xFFC;

// ============================================================================
// Flag Register (UART_FR) Bits  [ARM DDI 0183G page 3-8]
// ============================================================================

bitflags! {
    /// UART Flag Register bits
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct FlagRegister: u32 {
        /// Ring indicator
        const RI   = 1 << 8;
        /// Transmit FIFO empty
        const TXFE = 1 << 7;
        /// Receive FIFO full
        const RXFF = 1 << 6;
        /// Transmit FIFO full
        const TXFF = 1 << 5;
        /// Receive FIFO empty
        const RXFE = 1 << 4;
        /// UART busy
        const BUSY = 1 << 3;
        /// Data carrier detect
        const DCD  = 1 << 2;
        /// Data set ready
        const DSR  = 1 << 1;
        /// Clear to send
        const CTS  = 1 << 0;
    }
}

impl Default for FlagRegister {
    fn default() -> Self {
        // Default: TX FIFO empty, RX FIFO empty
        Self::TXFE | Self::RXFE
    }
}

// ============================================================================
// Control Register (UART_CR) Bits  [ARM DDI 0183G page 3-15]
// ============================================================================

bitflags! {
    /// UART Control Register bits
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ControlRegister: u32 {
        /// CTS hardware flow control enable
        const CTSEN  = 1 << 15;
        /// RTS hardware flow control enable
        const RTSEN  = 1 << 14;
        /// Complement of Out2 modem status output
        const OUT2   = 1 << 13;
        /// Complement of Out1 modem status output
        const OUT1   = 1 << 12;
        /// Request to send
        const RTS    = 1 << 11;
        /// Data transmit ready
        const DTR    = 1 << 10;
        /// Receive enable
        const RXE    = 1 << 9;
        /// Transmit enable
        const TXE    = 1 << 8;
        /// Loopback enable
        const LBE    = 1 << 7;
        /// UART enable
        const UARTEN = 1 << 0;
    }
}

impl Default for ControlRegister {
    fn default() -> Self {
        // Default: RXE and TXE enabled (as per ARM spec reset value 0x300)
        Self::RXE | Self::TXE
    }
}

// ============================================================================
// Line Control Register (UART_LCR_H) Bits  [ARM DDI 0183G page 3-12]
// ============================================================================

bitflags! {
    /// UART Line Control Register bits
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct LineControlRegister: u32 {
        /// Stick parity select
        const SPS  = 1 << 7;
        /// Word length bit 1 (WLEN[1])
        const WLEN1 = 1 << 6;
        /// Word length bit 0 (WLEN[0])
        const WLEN0 = 1 << 5;
        /// Enable FIFOs
        const FEN  = 1 << 4;
        /// Two stop bits select
        const STP2 = 1 << 3;
        /// Even parity select
        const EPS  = 1 << 2;
        /// Parity enable
        const PEN  = 1 << 1;
        /// Send break
        const BRK  = 1 << 0;
    }
}

impl Default for LineControlRegister {
    fn default() -> Self {
        Self::empty()
    }
}

impl LineControlRegister {
    /// Check if FIFO is enabled
    pub fn fifo_enabled(&self) -> bool {
        self.contains(Self::FEN)
    }
}

// ============================================================================
// Interrupt Bits (in UART_IMSC, UART_RIS, UART_MIS, UART_ICR)  
//     [ARM DDI 0183G page 3-18, 3-19, 3-20, 3-21]
// ============================================================================

bitflags! {
    /// UART Interrupt Status bits
    /// Used for IMSC (mask), RIS (raw), MIS (masked), ICR (clear)
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct InterruptBits: u32 {
        /// Overrun error interrupt
        const OE  = 1 << 10;
        /// Break error interrupt
        const BE  = 1 << 9;
        /// Parity error interrupt
        const PE  = 1 << 8;
        /// Framing error interrupt
        const FE  = 1 << 7;
        /// Receive timeout interrupt
        const RT  = 1 << 6;
        /// Transmit interrupt
        const TX  = 1 << 5;
        /// Receive interrupt
        const RX  = 1 << 4;
        /// nUARTDSR modem interrupt
        const DSR = 1 << 3;
        /// nUARTDCD modem interrupt
        const DCD = 1 << 2;
        /// nUARTCTS modem interrupt
        const CTS = 1 << 1;
        /// nUARTRI modem interrupt
        const RI  = 1 << 0;
    }
}

impl Default for InterruptBits {
    fn default() -> Self {
        Self::empty()
    }
}

// ============================================================================
// Data Register (UART_DR) Error Bits  [ARM DDI 0183G page 3-7]
// ============================================================================

bitflags! {
    /// UART Data Register bits
    /// Lower 8 bits contain the received data
    /// Upper bits contain error flags
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct DataRegisterError: u32 {
        /// Overrun error
        const OE = 1 << 11;
        /// Break error
        const BE = 1 << 10;
        /// Parity error
        const PE = 1 << 9;
        /// Framing error
        const FE = 1 << 8;
    }
}

// ============================================================================
// PL011 Identification Values
// ============================================================================

/// PL011 ARM Peripheral ID bytes
pub const PL011_PERIPH_ID: [u8; 4] = [0x11, 0x10, 0x14, 0x00];

/// PL011 PrimeCell ID bytes (standard for all PrimeCell peripherals)
pub const PL011_PCELL_ID: [u8; 4] = [0x0D, 0xF0, 0x05, 0xB1];

// ============================================================================
// Constants
// ============================================================================

/// FIFO depth when FIFO is enabled
pub const PL011_FIFO_DEPTH: usize = 16;

/// FIFO depth when FIFO is disabled (character mode)
pub const PL011_CHAR_DEPTH: usize = 1;

/// IBRD mask (16 bits)
pub const IBRD_MASK: u32 = 0xFFFF;

/// FBRD mask (6 bits)
pub const FBRD_MASK: u32 = 0x3F;

/// Default IFLS value (RX 1/2 full, TX 1/2 empty)
pub const DEFAULT_IFLS: u32 = 0x12;

