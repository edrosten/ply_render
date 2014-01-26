/* Copyright (C) Computer Vision Consulting, 2013.*/
#ifndef RENDERER_INDLUDE_STATIC_VECTOR_H
#define RENDERER_INDLUDE_STATIC_VECTOR_H

#include <array>
#include <stdexcept>
#include <cassert>
#include "debug_helpers.h"


//Class which behaves very much like std::vector
//except that memory is preallocated on the stack.
//This is useful for (e.g.) well behaved mesh models
//where an edge connects to always at most two faces.


template<class C, size_t Max> class static_vector
{
	private:
		std::array<C, Max> data;
		size_t num;	
	public:
		static_vector()
		:num(0)
		{}

		static_vector(const static_vector&)=default;
		static_vector& operator=(const static_vector&)=default;

		void push_back(C c)
		{	
			if(num == data.size())
				throw std::length_error("static_vector");
			data[num++] = c;
		}

		size_t size() const
		{
			return num;
		}

		const C& operator[](size_t i) const
		{
			assert(i < num);
			return data[i];
		}

		const C* begin() const
		{
			return data.begin();
		}

		const C* end() const
		{
			return data.begin() + num;
		}
		
		bool empty() const
		{
			return num == 0;
		}

		#ifdef DEBUG
			~static_vector()
			{
				for(size_t i=0;i<Max; i++)
					unset(data[i]);
				num=0;
			}
		#endif
};



#endif


