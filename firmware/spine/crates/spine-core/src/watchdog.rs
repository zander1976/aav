//! Command watchdog: trips when no valid command arrived for `timeout_ms`.

/// Times use a free-running millisecond counter that may wrap.
#[derive(Debug, Clone, Copy)]
pub struct Watchdog {
    timeout_ms: u32,
    last_ms: Option<u32>,
}

impl Watchdog {
    pub const fn new(timeout_ms: u32) -> Self {
        Watchdog {
            timeout_ms,
            last_ms: None,
        }
    }

    /// Record a valid command at `now_ms`.
    pub fn feed(&mut self, now_ms: u32) {
        self.last_ms = Some(now_ms);
    }

    /// True until the first command, and whenever the last one is too old.
    pub fn expired(&self, now_ms: u32) -> bool {
        match self.last_ms {
            None => true,
            Some(t) => now_ms.wrapping_sub(t) > self.timeout_ms,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expired_before_first_command() {
        assert!(Watchdog::new(100).expired(0));
    }

    #[test]
    fn trips_after_timeout() {
        let mut w = Watchdog::new(100);
        w.feed(1000);
        assert!(!w.expired(1100));
        assert!(w.expired(1101));
    }

    #[test]
    fn survives_counter_wrap() {
        let mut w = Watchdog::new(100);
        w.feed(u32::MAX - 20);
        assert!(!w.expired(50)); // 71 ms later, after the wrap
        assert!(w.expired(100)); // 121 ms later
    }
}
