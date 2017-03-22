// Credits:   @rpavlik for writing this header.
// Note:  "Importantly, it includes a header to both indicate to the Arduino IDE
//         that this library should be used, and set up the preprocessor environment
//         so the ample #define lines in the Arduino and AVR libraries don't wreak havoc with Eigen.

// Disable debug asserts.
#define EIGEN_NO_DEBUG 1

// Hint to number of registers
#define EIGEN_ARCH_DEFAULT_NUMBER_OF_REGISTERS 16

#ifdef A0
# define NEED_A0_RESTORED A0
# undef A0
#endif
#ifdef A1
# define NEED_A1_RESTORED A1
# undef A1
#endif
#ifdef B0
# define NEED_B0_RESTORED B0
# undef B0
#endif
#ifdef B1
# define NEED_B1_RESTORED B1
# undef B1
#endif

namespace std {
	struct nothrow_t;
}


// Include main EIGEN Core header
#include <Eigen/Core>


#ifdef NEED_A0_RESTORED
# define A0 NEED_A0_RESTORED
# undef NEED_A0_RESTORED
#endif
#ifdef NEED_A1_RESTORED
# define A1 NEED_A1_RESTORED
# undef NEED_A1_RESTORED
#endif
#ifdef NEED_B0_RESTORED
# define B0 NEED_B0_RESTORED
# undef NEED_B0_RESTORED
#endif
#ifdef NEED_B1_RESTORED
# define B1 NEED_B1_RESTORED
# undef NEED_B1_RESTORED
#endif
