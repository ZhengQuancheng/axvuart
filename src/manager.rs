//! Simple UART Manager for Guest-to-Guest Communication

use alloc::collections::BTreeMap;
use alloc::sync::Arc;
use spin::Mutex;

use crate::VirtPL011;

static UART_REGISTRY: Mutex<BTreeMap<usize, Arc<VirtPL011>>> = Mutex::new(BTreeMap::new());
static UART_COUNTER: Mutex<usize> = Mutex::new(1);
pub struct UartManager;

impl UartManager {
    /// Register a UART device for a specific VM.
    pub fn register(uart_id: usize, uart: Arc<VirtPL011>) {
        let mut registry = UART_REGISTRY.lock();
        registry.insert(uart_id, uart);
    }

    /// Get the UART device for a specific VM.
    pub fn get(uart_id: usize) -> Option<Arc<VirtPL011>> {
        let registry = UART_REGISTRY.lock();
        registry.get(&uart_id).cloned()
    }
    
    /// Unregister the UART device for a specific VM.
    #[allow(unused)]
    pub fn unregister(uart_id: usize) {
        let mut registry = UART_REGISTRY.lock();
        registry.remove(&uart_id);
    }

    /// Connect two VMs' UARTs for communication.
    #[allow(unused)]
    pub fn connect(uart_id_a: usize, uart_id_b: usize) {
        let registry = UART_REGISTRY.lock();
        if let (Some(uart_a), Some(uart_b)) = (registry.get(&uart_id_a), registry.get(&uart_id_b)) {
            // Use Arc::downgrade to create Weak references
            uart_a.set_peer(Arc::downgrade(uart_b));
            uart_b.set_peer(Arc::downgrade(uart_a));
            info!("Connected UARTs of UART {} and UART {}", uart_id_a, uart_id_b);
        } else {
            warn!("One or both UARTs not found for {} and {}", uart_id_a, uart_id_b);
        }
    }

    pub fn generate_uart_id() -> usize {
        let mut counter = UART_COUNTER.lock();
        let id = *counter;
        *counter += 1;
        id
    }
}
  