AC_INIT(,1.0)
AC_COPYRIGHT()


orig_CXXFLAGS="$CXXFLAGS"

AC_PROG_AWK
AC_PROG_CXX
AC_PROG_CC

AC_LANG(C++)

################################################################################
#
# Useful macros
#

dnl APPEND(var, value)
dnl This appends vale to a shell variable var
define(APPEND, [$1="$$1 $2"])



dnl TEST_AND_SET_CXXFLAG(flag, [program], [run])
dnl
dnl This attempts to compile a and run program with a certain compiler flag.
dnl If no program is given, then the minimal C++ program is compiled, and 
dnl this tests just the validity of the compiler flag. 
dnl
define([TEST_AND_SET_CXXFLAG],[
	if test "$3" == ""
	then
		AC_MSG_CHECKING([if compiler flag $1 works])	
	else
		AC_MSG_CHECKING([$3])	
	fi
	save_CXXFLAGS="$CXXFLAGS"
	APPEND(CXXFLAGS, [$1])

	m4_if([$2],[],[pushdef(prog, [int main(){}])], [pushdef(prog, [$2])])

	m4_if([$4],[run],
		[AC_RUN_IFELSE([AC_LANG_SOURCE([prog])], [cvd_conf_test=1],[cvd_conf_test=0], [cvd_conf_test=0])],
		[AC_COMPILE_IFELSE([AC_LANG_SOURCE([prog])], [cvd_conf_test=1],[cvd_conf_test=0])]
	)
	
	
	popdef([prog])

	if test $cvd_conf_test = 1
	then
		AC_MSG_RESULT(yes)
		ts_success=yes
	else
		AC_MSG_RESULT(no)
		CXXFLAGS="$save_CXXFLAGS"
		ts_success=no
	fi
])


TEST_AND_SET_CXXFLAG(-Wall)
TEST_AND_SET_CXXFLAG(-Wextra)
TEST_AND_SET_CXXFLAG(-W)
TEST_AND_SET_CXXFLAG(-ansi)
TEST_AND_SET_CXXFLAG(-pedantic)
TEST_AND_SET_CXXFLAG(-std=c++11)
TEST_AND_SET_CXXFLAG(-std=c++0x)
################################################################################
#
# Check for TooN
#
toon=yes
AC_CHECK_HEADERS(TooN/TooN.h, [], [toon=no])

comp=no
AC_MSG_CHECKING([TooN])
AC_COMPILE_IFELSE([AC_LANG_SOURCE([
	#include <TooN/TooN.h>
	int main(){TooN::Vector<2> v;return 0;}
])], [comp=yes], [toon=no])

AC_MSG_RESULT($comp)

if test $toon = no
then
	AC_MSG_ERROR([A working copy of TooN is required for this program.
Get TooN here: http://www.edwardrosten.com/cvd/toon.html
])
fi


################################################################################
#
# Check for libcvd
#
cvd=yes
AC_CHECK_HEADERS(cvd/image_io.h, [], [cvd=no])

AC_MSG_CHECKING(libCVD)

LDFLAGS="$LDFLAGS -lcvd"

comp=no
AC_COMPILE_IFELSE([AC_LANG_SOURCE([
	#include <cvd/image_io.h>
	int main(){ CVD::Image<CVD::byte> i = CVD::img_load(""); return 0;}
])], [comp=yes], [comp=no])

AC_MSG_RESULT([$comp])

if test $comp = no
then
	AC_MSG_CHECKING(If libcvd needs -msse2)
	APPEND(CXXFLAGS,[ -msse2])

	AC_COMPILE_IFELSE([AC_LANG_SOURCE([
		#include <cvd/image_io.h>
		int main(){ CVD::Image<CVD::byte> i = CVD::img_load(""); return 0;}
	])], [comp=yes], [comp=no])
	AC_MSG_RESULT($comp)
fi

if test $comp = no
then
	AC_MSG_ERROR([A working copy of libCVD is required for this program.
Get libCVD here: http://www.edwardrosten.com/cvd/
])
fi


AC_OUTPUT(Makefile)
