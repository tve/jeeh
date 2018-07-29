# JeeH

Headers for framework- and platform-agnostic code.

* This code can be used in combination with arduino, libopencm3, mbed, and other frameworks.
* A good C++ compiler with C++11 support is essential, due to heavy use of C++ templates.
* The resulting code is small (_really small_, sometimes) and fast (_really fast_, sometimes).
* Everything here is in flux. API's will change, as better idioms are being discovered.
* This library is intended for use with [PlatformIO](https://platformio.org/lib/show/3082/JeeH): just add `JeeH` to `lib_deps`.

### TvE's fork

This fork of jeelabs/jeeh has code and customizations used by TvE. Enhancements are meant to be eventually merged back into the upstream, but some changes are there because of differences in environments and boards used.

### Examples

See the [Sampler](https://github.com/jeelabs/jeeh/tree/master/examples/sampler) project.

### License

JeeH wants to stand on the shoulders of giants. As every giant tends to prefer
its own license scheme (that's one of the perks of being a giant), JeeH bows in
total worship for each them and lives in the public domain. Do as you wish.
