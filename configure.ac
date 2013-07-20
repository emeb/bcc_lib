#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ([2.60])
AC_INIT([bcc_lib], [1.0], [BUG-REPORT-ADDRESS])
AM_INIT_AUTOMAKE([1.10 no-define foreign])
AC_CONFIG_SRCDIR([bcc_lib.h])
AC_CONFIG_HEADERS([config.h])

AC_CONFIG_MACRO_DIR([config])

# Checks for programs.
AC_PROG_CC

# Define Version Numbers
AC_SUBST([BCC_LIB_SO_VERSION], [1:0:0])
AC_SUBST([BCC_LIB_API_VERSION], [1.0])

####################
### Automake
####################
#AM_INIT_AUTOMAKE([1.9])
LT_INIT([disable-static])

# Checks for libraries.
AC_CHECK_LIB([m], [exp])

# Checks for header files.
AC_CHECK_HEADERS([fcntl.h stdint.h stdlib.h string.h sys/ioctl.h unistd.h])

AC_CHECK_HEADER([linux/spi/spidev.h],
                [],
                [AC_MSG_ERROR([Missing SPI, please install it])])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_INLINE
AC_TYPE_UINT32_T
AC_TYPE_UINT8_T

# Checks for library functions.

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
