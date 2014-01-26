/* Copyright (C) Computer Vision Consulting, 2013.*/
#ifndef RENDERER_INC_POINTER_SET_H
#define RENDERER_INC_POINTER_SET_H

#include <vector>

// The following class behaves identically to a set<X*>, with the 
// constraint that all the X*'s have to be drawn from the same
// std::vector.
//
// In comparison to using unordered_set and set, it's about 6 and 8 times
// faster in the benchmarks, respectively. Without the speedup, the set
// access was dominating the rendering time.
//
// So it's very much not premature optimization :)
//
template<class C>
class PointerSet
{
	private:

		struct Block
		{
			typedef uint64_t type;
			static const int Bits = 64;
			type data;

			void do_clear()
			{
				data = 0;
			}

			bool is_empty() const
			{
				return !data;
			}
			
			bool get(int bit)const
			{
				type mask = type(1) << bit;
				return data & mask;
			}

			void flip(int bit)
			{
				type mask = type(1) << (bit);
				data^=mask;
			}

			void set(int bit)
			{
				type mask = type(1) << (bit);
				data|=mask;
			}

			bool erase(int bit)
			{
				type mask = type(1) << bit;
				bool n = data & mask;
				data &= ~mask;
				return n;
			}

		};


		
		class iterator
		{
			public:
				typedef struct forward_iterator_tag iterator_category;
				typedef unsigned int value_type;
				typedef std::ptrdiff_t difference_type;
				typedef const int reference;
				typedef void pointer;

				bool operator==(const iterator& i) const
				{
					return block == i.block && bit == i.bit;
				}

				bool operator!=(const iterator& i) const
				{
					return ! (*this==i);
				}
				
				iterator(const PointerSet& fs, bool end)
				:s(fs)
				{
					if(end)
					{
						block = s.blocks.size();
						bit=0;
					}
					else
					{
						block=0;
						find_next_block_and_bit();
					}
				}

				void find_next_block_and_bit()
				{

					//Find the first non-empty block
					for(; block < s.blocks.size() && s.blocks[block].is_empty(); block++)
					{}
					bit=0;

					//Find the first set bit.
					if(block != s.blocks.size())
						for(bit=0; bit<Block::Bits; bit++)
							if(s.blocks[block].get(bit))
								break;

					assert(bit != Block::Bits);
				}
				
				void operator++()
				{
					bit++;
					//Find the next bit in the current block
					for(; bit<Block::Bits; bit++)
					{
						if(s.blocks[block].get(bit))
							return;
					}
					
					block++;
					//else we're onto the noxt block
					find_next_block_and_bit();
				}
				
				
				const C* operator*()
				{
					return (bit + block * Block::Bits) + s.base;
				}
				
			private:
				const PointerSet& s;
				unsigned int block;
				int bit;

		};

		
		const C* base;

	public:

		std::vector<Block> blocks;

		iterator begin()
		{
			return iterator(*this, false);
		}

		iterator end()
		{
			return iterator(*this, true);
		}


		PointerSet(const std::vector<C>& v)
		:base(v.data())
		{
			int sz = v.size();

			blocks.resize((sz + Block::Bits-1) / Block::Bits);

			for(auto& b:blocks)
				b.do_clear();
		}
		

		void clear()
		{
			for(auto& b:blocks)
				b.do_clear();
		}

		void insert(const C* c)
		{
			int n = c - base;
			int block = n / Block::Bits;
			int bit = n % Block::Bits;

			blocks[block].set(bit);
		}
		
		void flip(const C* c)
		{
			int n = c - base;
			int block = n / Block::Bits;
			int bit = n % Block::Bits;

			blocks[block].flip(bit);
		}

		int count(const C* c)
		{
			int n = c - base;
			int block = n / Block::Bits;
			int bit = n % Block::Bits;
			return blocks[block].get(bit);
		}

		int erase(const C* c)
		{
			int n = c - base;
			int block = n / Block::Bits;
			int bit = n % Block::Bits;
			return blocks[block].erase(bit);
		}

};

#endif
