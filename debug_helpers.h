#ifndef RENDERER_INDLUDE_DEBUG_HELPERS_H
#define RENDERER_INDLUDE_DEBUG_HELPERS_H

	#include <TooN/TooN.h>

	//Some helper functions to fill things with funny values
	//on destruction to help catch bugs.
	#ifdef DEBUG
		template<class C> void unset(C& c)
		{
			c=C();
		}
		template<class C*> void unset(C& c)
		{
			c = reinterpret_cast<C>(0xbadc0ffee0ddf00d);
		}

		template<int I> void unset(TooN::Vector<I>& v)
		{
			for(int i=0; i < I; i++)
				v[i] = -6.005360053e99;
		}
	#else
		template<class C> void unset(C& c)
		{}

	#endif
#endif
