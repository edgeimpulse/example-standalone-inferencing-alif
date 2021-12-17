# Arm(R) Ethos(TM)-U core software

## Building

The Arm(R) Ethos(TM)-U core software is built with CMake. It is recommended to
build out of tree like illustrated below.

```
$ mkdir build
$ cd build
$ cmake .. -DCMAKE_TOOLCHAIN_FILE=<toolchain> -DCMAKE_SYSTEM_PROCESSOR=cortex-m<nr><features>
$ make
```

Available build options can be listed with `cmake -LH ..`.

Supported CPU targets are any of the Cortex(R)-M processors with any of the
supported features, for example cortex-m33+nodsp+nofp. A toolchain file is
required to cross compile the software.

# Contributions

The Arm Ethos-U project welcomes contributions under the Apache-2.0 license.

Before we can accept your contribution, you need to certify its origin and give
us your permission. For this process we use the Developer Certificate of Origin
(DCO) V1.1 (https://developercertificate.org).

To indicate that you agree to the terms of the DCO, you "sign off" your
contribution by adding a line with your name and e-mail address to every git
commit message. You must use your real name, no pseudonyms or anonymous
contributions are accepted. If there are more than one contributor, everyone
adds their name and e-mail to the commit message.

```
Author: John Doe \<john.doe@example.org\>
Date:   Mon Feb 29 12:12:12 2016 +0000

Title of the commit

Short description of the change.

Signed-off-by: John Doe john.doe@example.org
Signed-off-by: Foo Bar foo.bar@example.org
```

The contributions will be code reviewed by Arm before they can be accepted into
the repository.

# Security

Please see [Security](SECURITY.md).

# License

The Arm Ethos-U core software is provided under an Apache-2.0 license. Please
see [LICENSE.txt](LICENSE.txt) for more information.

# Trademark notice

Arm, Cortex and Ethos are registered trademarks of Arm Limited (or its
subsidiaries) in the US and/or elsewhere.
