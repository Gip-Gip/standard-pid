## Implemention of the industry standard PID algorithm for serious embedded projects

PID is a ubiquitous form of output control used very commonly in all sorts of
embedded applications. The goal of this library is to provide the standard
PID implementation found across most industrial devices and allows for
compatability across controllers and with autotuning algorithms.

The core struct also allows a few niceties to combat common PID issues and
is time aware, as in you supply the delta-time between updates to ensure
reliable computation and cross-device compatability.

This crate is entirely `no_std` and is intended to be used with your embedded project.

See [Wikipedia](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Alternative_nomenclature_and_forms)
for more information regarding standard form PID.

