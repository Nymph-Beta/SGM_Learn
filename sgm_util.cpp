#include "stdafx.h"
#include "sgm_util.h"
#include <string>
#include <algorithm>
#include <cassert>
#include <vector>
#include <queue>
#include <iostream>

void sgm_util::census_transform_5x5(const uint8* source, uint32* census, const sint32& width,
	const sint32& height)
{
	// Check if source or census pointers are null or if the width or height is less than or equal to 5
	if (source == nullptr || census == nullptr || width <= 5 || height <= 5) {
		return;
	}

	// Compute census values for each pixel
	for (sint32 i = 2; i < height - 2; i++) {
		for (sint32 j = 2; j < width - 2; j++) {

			// Get the value of the center pixel
			const uint8 gray_center = source[i * width + j];
			//std::cout << "Gray center value at (" << i << ", " << j << "): " << static_cast<int>(gray_center) << std::endl;

			// Initialize census value
			uint32 census_val = 0u;

			// Compare the value of each neighboring pixel in a 5x5 window with the center pixel value
			for (sint32 r = -2; r <= 2; r++) {
                for (sint32 c = -2; c <= 2; c++) {
                    census_val <<= 1;
                    const uint8 gray = source[(i + r) * width + j + c];
                    //std::cout << "Gray value: " << static_cast<int>(gray) << std::endl;

					// If the neighboring pixel value is less than the center pixel value, set the corresponding bit to 1
                    if (gray < gray_center) {
                        census_val += 1;
                    }


					//std::cout << "census_val: " << census_val << std::endl;
                }

			}

			// Store the census value for the center pixel
			census[i * width + j] = census_val;

		}
	}

	// ´òÓ¡censusÖµ
	//for (sint32 i = 2; i < height - 2; i++) {
	//	for (sint32 j = 2; j < width - 2; j++) {
	//		uint32 census_val = census[i * width + j];
	//		std::cout << "Census value at (" << i << ", " << j << "): " << census_val << std::endl;
	//	}
	//}
}

uint8 sgm_util::Hamming32(const uint32& x, const uint32& y)
{
	uint32 dist = 0, val = x ^ y;	// Compute the bitwise XOR of x and y

	// Count the number of set bits
	while (val) {
		++dist;		// Increment the distance count
		val &= val - 1;		// Remove the rightmost set bit
	}

	// Return the Hamming distance as an 8 - bit unsigned integer
	return static_cast<uint8>(dist);
}

uint8 sgm_util::Hamming64(const uint64& x, const uint64& y)
{
	uint64 dist = 0, val = x ^ y;

	// Count the number of set bits
	while (val) {
		++dist;
		val &= val - 1;
	}

	return static_cast<uint8>(dist);
}
