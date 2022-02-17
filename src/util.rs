// SPDX-License-Identifier: GPL-3.0-or-later


pub struct SharedWithInterrupt<T>(T);
impl<T> SharedWithInterrupt<T> {
    pub fn new(v: T) -> Self {
        Self(v)
    }
    pub fn lock<R>(&mut self, mut f: impl FnMut(&mut T) -> R) -> R {
        cortex_m::interrupt::free(|_| f(&mut self.0))
    }

    pub unsafe fn lock_from_interrupt<R>(&mut self, mut f: impl FnMut(&mut T) -> R) -> R {
        f(&mut self.0)
    }
}
