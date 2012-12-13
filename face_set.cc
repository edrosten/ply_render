#include <vector>
#include <set>
#include <iostream>
#include <iterator>
#include <cstdint>
#include <cassert>
#include <limits>

using namespace std;

class FaceSet
{
	private:

		struct Block
		{
			static const int ChunkBits = 64;
			static const int N=4;
			static const int Bits = N * ChunkBits;
			uint64_t data[N];

			bool lazy_clear;
			int num_empty_chunx;

			void do_clear()
			{
				for(int i=0; i < N; i++)
					data[i] = 0;

				lazy_clear=false;
				num_empty_chunx=N;
			}

			bool is_empty() const
			{
				return (lazy_clear || num_empty_chunx == N);
			}
			
			bool get(int bit)const
			{
				int chunk = bit/ChunkBits;
				uint64_t mask = uint64_t(1) << (bit%ChunkBits);
				return data[chunk] & mask;
			}

			void flip(int bit)
			{
				int chunk = bit/ChunkBits;
				uint64_t mask = uint64_t(1) << (bit%ChunkBits);

				if(lazy_clear)
				{
					do_clear();
					data[chunk] = mask;
					num_empty_chunx--;
				}
				else
				{
					if(data[chunk] == 0)
					{
						data[chunk]=mask;
						num_empty_chunx--;
					}
					else if(data[chunk] == mask)
					{
						data[chunk]=0;
						num_empty_chunx++;
					}
					else
						data[chunk]^= mask;
				}
			}

		};


		
		class iterator
		{
			public:
				typedef struct forward_iterator_tag iterator_category;
				typedef unsigned int value_type;
				typedef ptrdiff_t difference_type;
				typedef const int reference;
				
				iterator(const FaceSet fs, bool end)
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
					//Find the next bit in the current block
					for(bit=0; bit<Block::Bits; bit++)
						if(s.blocks[block].get(bit))
							return;

					//else we're onto the noxt block
					find_next_block_and_bit();
				}
				
				
				int operator*()
				{
					return bit + block * Block::Bits;
				}
				
			private:
				const FaceSet& s;
				unsigned int block;
				int bit;

		};


	public:

		vector<Block> blocks;

		FaceSet(int sz)
		{
			blocks.resize((sz + Block::Bits-1) / Block::Bits);

			for(auto& b:blocks)
				b.do_clear();
		}
		

		void clear()
		{
			for(auto& b:blocks)
				b.lazy_clear=true;
		}
		
		void flip(int n)
		{
			int block = n / Block::Bits;
			int bit = n % Block::Bits;

			blocks[block].flip(bit);
		}

};
