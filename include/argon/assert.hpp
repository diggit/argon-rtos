
#ifndef _AR_ASSERT_HDR_INCLUDED
#define _AR_ASSERT_HDR_INCLUDED

#include <cassert>

// passing through assert, always evaluates expression
#ifdef NDEBUG /* required by ANSI standard */
	#define assertWrap(__e) (__e)
	#define assertEval(__e) (__e)
#else
	// buffer to __res to avoid multiple evaluations
	#define assertWrap(__e) [](auto __res){if(!__res) {__assert_func(__FILE__, __LINE__, __ASSERT_FUNC, "");} return __res;}(__e)
	#define assertEval(__e) assertWrap(__e)
#endif



#endif
