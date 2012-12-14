#ifndef RENDERER_INDLUDE_DEBUG_ARRAY_H
#define RENDERER_INDLUDE_DEBUG_ARRAY_H
	
	//Array behaves exctly like std::array
	//except that in debug mode it does
	//bounds checking and trashes memory on 
	//destruction

	#include "debug_helpers.h"


	#ifdef DEBUG
	template<class C, size_t Size> class Array: private std::array<C, Size>
	{
		public:
			using std::array<C, Size>::size;
			using std::array<C, Size>::begin;
			using std::array<C, Size>::end;
			using std::array<C, Size>::value_type;

			C& operator[](size_t i)
			{
				assert(i < Size);
				return std::array<C, Size>::operator[](i);
			}	

			const C& operator[](size_t i) const
			{
				assert(i < Size);
				return std::array<C, Size>::operator[](i);
			}

			~Array()
			{
				for(size_t i=0; i < Size; i++)
				{
					unset(operator[](i));
				}
			}
	};

	#else
		template<class C, size_t Size> using Array = std::array<C, Size>;
	#endif

#endif

