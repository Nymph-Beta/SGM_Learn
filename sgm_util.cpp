#include "stdafx.h"
#include "sgm_util.h"
#include <string>
#include <algorithm>
#include <cassert>
#include <vector>
#include <queue>
#include <iostream>

void sgm_util::census_transform_5x5(const uint8* source, uint32* census, const sint32& width,
	const sint32& height){
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

	
	//for (sint32 i = 2; i < height - 2; i++) {
	//	for (sint32 j = 2; j < width - 2; j++) {
	//		uint32 census_val = census[i * width + j];
	//		std::cout << "Census value at (" << i << ", " << j << "): " << census_val << std::endl;
	//	}
	//}
}

uint8 sgm_util::Hamming32(const uint32& x, const uint32& y){
	uint32 dist = 0, val = x ^ y;	// Compute the bitwise XOR of x and y

	// Count the number of set bits
	while (val) {
		++dist;		// Increment the distance count
		val &= val - 1;		// Remove the rightmost set bit
	}

	// Return the Hamming distance as an 8 - bit unsigned integer
	return static_cast<uint8>(dist);
}

uint8 sgm_util::Hamming64(const uint64& x, const uint64& y){
	uint64 dist = 0, val = x ^ y;

	// Count the number of set bits
	while (val) {
		++dist;
		val &= val - 1;
	}

	return static_cast<uint8>(dist);
}

void sgm_util::CostAggregateLeftRight(const uint8* img_data, const sint32& width, const sint32& height,
	const sint32& min_disparity, const sint32& max_disparity, const sint32& p1, const sint32& p2_init,
	const uint8* cost_init, uint8* cost_aggr, bool is_forward) {

	// Check if img_data, cost_init, or cost_aggr pointers are null
	if (img_data == nullptr || cost_init == nullptr || cost_aggr == nullptr) {
		return;
	}
	assert(width > 0 && height > 0 && min_disparity >= 0 && max_disparity >= min_disparity);

	// Compute the disparity range
	const sint32 disp_range = max_disparity - min_disparity;

	// Reference passing of P1 and P2 initialization penalty parameters, used for path cost calculation
	const auto& P1 = p1;
	const auto& P2 = p2_init;

	// Determine traversal direction. If is_forward is true, direction is 1 (left to right), otherwise -1 (right to left)
	const sint32 direction = is_forward ? 1 : -1;

	// Start aggregation, traverse each row of the image
	for (sint32 i = 0u; i < height; i++) {

		// Calculate the starting positions of the initial cost and aggregated cost for the current row based on the direction
		auto cost_init_row = (is_forward) ? cost_init + i * width * disp_range : cost_init + i * width * disp_range + (width - 1) * disp_range;
		auto cost_aggr_row = (is_forward) ? cost_aggr + i * width * disp_range : cost_aggr + i * width * disp_range + (width - 1) * disp_range;
		auto img_row = (is_forward) ? img_data + i * width : img_data + i * width + (width - 1);

		// Get the grayscale value of the current pixel
		uint8 gray = *img_row;
		uint8 gray_last = *img_row; // Initialize the grayscale value of the previous pixel

		// Initialize the cost array for the previous path, with size disp_range + 2, and initial value UINT8_MAX (infinity)
		std::vector<uint8>cost_last_path(disp_range + 2, UINT8_MAX);

		// Copy the initial cost of the current row into the aggregated cost array
		memcpy(cost_aggr_row, cost_init_row, disp_range * sizeof(uint8));
		// Copy the current aggregated cost into the cost array of the previous path (starting from index 1)
		memcpy(&cost_last_path[1], cost_aggr_row, disp_range * sizeof(uint8));

		// Update pointers to point to the next pixel
		cost_init_row += direction * disp_range;
		cost_aggr_row += direction * disp_range;
		img_row += direction;

		// Calculate the minimum cost on the current path, used for subsequent calculations
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path)
			mincost_last_path = std::min(mincost_last_path, cost);

		// Traverse each pixel in the current row
		for (sint32 j = 0; j < width - 1; j++) {
			gray = *img_row; // Get the grayscale value of the current pixel
			uint8 min_cost = UINT8_MAX; // Initialize the minimum cost of the current pixel
			const sint32 disp_offset = j * disp_range; // Calculate disparity offset

			// Compute the cost of the remaining disparities
			for (sint32 d = 0; d < disp_range; d++) {
				const uint8 cost = cost_init_row[d]; // Initial cost of the current disparity
				const uint16 l1 = cost_last_path[d + 1]; // Cost of the previous disparity
				const uint16 l2 = cost_last_path[d] + P1; // Cost of the next disparity plus penalty P1
				const uint16 l3 = cost_last_path[d + 2] + P1; // Cost of the next disparity plus penalty P1
				const uint16 l4 = mincost_last_path + std::max(P1, P2 / (abs(gray - gray_last) + 1)); // Minimum cost plus dynamic penalty

				// Calculate the aggregated cost of the current disparity
				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				// Update the aggregated cost array and the minimum cost of the current pixel
				cost_aggr_row[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// Update the minimum cost of the previous path
			mincost_last_path = min_cost;
			// Copy the current aggregated cost into the cost array of the previous path (starting from index 1)
			memcpy(&cost_last_path[1], cost_aggr_row, disp_range * sizeof(uint8));

			// Update the pointers
			cost_init_row += direction * disp_range;
			cost_aggr_row += direction * disp_range;
			img_row += direction;

			gray_last = gray;
		}
	}
}

void sgm_util::CostAggregateUpDown(const uint8* img_data, const sint32& width, const sint32& height,
	const sint32& min_disparity, const sint32& max_disparity, const sint32& p1, const sint32& p2_init,
	const uint8* cost_init, uint8* cost_aggr, bool is_forward) {

	// Check if img_data, cost_init, or cost_aggr pointers are null
	if (img_data == nullptr || cost_init == nullptr || cost_aggr == nullptr) {
		return;
	}
	assert(width > 0 && height > 0 && min_disparity >= 0 && max_disparity >= min_disparity);

	// Compute the disparity range
	const sint32 disp_range = max_disparity - min_disparity;

	const auto& P1 = p1;
	const auto& P2 = p2_init;

	const sint32 direction = is_forward ? 1 : -1;

	for (sint32 j = 0u; j < width; j++) {
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		uint8 gray = *img_col;
		uint8 gray_last = *img_col;

		std::vector<uint8>cost_last_path(disp_range + 2, UINT8_MAX);

		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(uint8));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));
		cost_init_col += direction * width * disp_range;
		cost_aggr_col += direction * width * disp_range;
		img_col += direction * width;

		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path)
			mincost_last_path = std::min(mincost_last_path, cost);

		for (sint32 i = 0; i < height - 1; i++) {
			gray = *img_col;
			uint8 min_cost = UINT8_MAX;
			const sint32 disp_offset = j * disp_range;

			// Compute the cost of the remaining disparities
			for (sint32 d = 0; d < disp_range; d++) {
				const uint8 cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2 / (abs(gray - gray_last) + 1));

				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

			// Update the pointers
			cost_init_col += direction * width * disp_range;
			cost_aggr_col += direction * width * disp_range;
			img_col += direction * width;

			gray_last = gray;
		}
	}
}

void sgm_util::CostAggregateDagonal_1(const uint8* img_data, const sint32& width, const sint32& height,
	const sint32& min_disparity, const sint32& max_disparity, const sint32& p1, const sint32& p2_init,
	const uint8* cost_init, uint8* cost_aggr, bool is_forward)
{
	assert(width > 1 && height > 1 && max_disparity > min_disparity);

	// Disparity range
	const sint32 disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// Forward (top left to bottom right): is_forward = true; direction = 1
	// Backward (bottom right to top left): is_forward = false; direction = -1;
	const sint32 direction = is_forward ? 1 : -1;

	// Aggregation

	// Store the current row and column numbers, and check if the image boundary is reached
	sint32 current_row = 0;
	sint32 current_col = 0;

	for (sint32 j = 0; j < width; j++) {
		// The head of the path is the first (or last, dir=-1) row pixel of each column
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		// Cost array of the previous pixel on the path, with two additional elements to avoid boundary overflow (one more at the beginning and end)
		std::vector<uint8> cost_last_path(disp_range + 2, UINT8_MAX);

		// Initialization: the aggregated cost of the first pixel is equal to the initial cost
		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(uint8));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

		// Current and previous grayscale values on the path
		uint8 gray = *img_col;
		uint8 gray_last = *img_col;

		// The next pixel on the diagonal path, separated by width + 1 pixels
		// Additional boundary handling is required here
		// When moving along the diagonal, if the image column boundary is reached, the row number continues in the original direction, and the column number jumps to the other boundary
		current_row = is_forward ? 0 : height - 1;
		current_col = j;
		if (is_forward && current_col == width - 1 && current_row < height - 1) {
			// Top left to bottom right, reaching the right boundary
			cost_init_col = cost_init + (current_row + direction) * width * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
			img_col = img_data + (current_row + direction) * width;
			current_col = 0;
		}
		else if (!is_forward && current_col == 0 && current_row > 0) {
			// Bottom right to top left, reaching the left boundary
			cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			img_col = img_data + (current_row + direction) * width + (width - 1);
			current_col = width - 1;
		}
		else {
			cost_init_col += direction * (width + 1) * disp_range;
			cost_aggr_col += direction * (width + 1) * disp_range;
			img_col += direction * (width + 1);
		}

		// Minimum cost of the previous pixel on the path
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// Aggregation starts from the second pixel in the direction
		for (sint32 i = 0; i < height - 1; i++) {
			gray = *img_col;
			uint8 min_cost = UINT8_MAX;
			for (sint32 d = 0; d < disp_range; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
				const uint8  cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));

				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// Reset the minimum cost and cost array of the previous pixel
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

			// Row and column numbers of the current pixel
			current_row += direction;
			current_col += direction;

			// Next pixel, additional boundary handling is required here
			// When moving along the diagonal, if the image column boundary is reached, the row number continues in the original direction, and the column number jumps to the other boundary
			if (is_forward && current_col == width - 1 && current_row < height - 1) {
				// Top left to bottom right, reaching the right boundary
				cost_init_col = cost_init + (current_row + direction) * width * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
				img_col = img_data + (current_row + direction) * width;
				current_col = 0;
			}
			else if (!is_forward && current_col == 0 && current_row > 0) {
				// Bottom right to top left, reaching the left boundary
				cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				img_col = img_data + (current_row + direction) * width + (width - 1);
				current_col = width - 1;
			}
			else {
				cost_init_col += direction * (width + 1) * disp_range;
				cost_aggr_col += direction * (width + 1) * disp_range;
				img_col += direction * (width + 1);
			}

			// Reassign pixel value
			gray_last = gray;
		}
	}
}

void sgm_util::CostAggregateDagonal_2(const uint8* img_data, const sint32& width, const sint32& height,
	const sint32& min_disparity, const sint32& max_disparity, const sint32& p1, const sint32& p2_init,
	const uint8* cost_init, uint8* cost_aggr, bool is_forward)
{
	assert(width > 1 && height > 1 && max_disparity > min_disparity);

	// Disparity range
	const sint32 disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// Forward (top right to bottom left): is_forward = true; direction = 1
	// Backward (bottom left to top right): is_forward = false; direction = -1;
	const sint32 direction = is_forward ? 1 : -1;

	// Aggregation

	// Store the current row and column numbers, and check if the image boundary is reached
	sint32 current_row = 0;
	sint32 current_col = 0;

	for (sint32 j = 0; j < width; j++) {
		// The head of the path is the first (or last, dir=-1) row pixel of each column
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		// Cost array of the previous pixel on the path, with two additional elements to avoid boundary overflow (one more at the beginning and end)
		std::vector<uint8> cost_last_path(disp_range + 2, UINT8_MAX);

		// Initialization: the aggregated cost of the first pixel is equal to the initial cost
		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(uint8));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

		// Current and previous grayscale values on the path
		uint8 gray = *img_col;
		uint8 gray_last = *img_col;

		// The next pixel on the diagonal path, separated by width - 1 pixels
		// Additional boundary handling is required here
		// When moving along the diagonal, if the image column boundary is reached, the row number continues in the original direction, and the column number jumps to the other boundary
		current_row = is_forward ? 0 : height - 1;
		current_col = j;
		if (is_forward && current_col == 0 && current_row < height - 1) {
			// Top right to bottom left, reaching the left boundary
			cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			img_col = img_data + (current_row + direction) * width + (width - 1);
			current_col = width - 1;
		}
		else if (!is_forward && current_col == width - 1 && current_row > 0) {
			// Bottom left to top right, reaching the right boundary
			cost_init_col = cost_init + (current_row + direction) * width * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
			img_col = img_data + (current_row + direction) * width;
			current_col = 0;
		}
		else {
			cost_init_col += direction * (width - 1) * disp_range;
			cost_aggr_col += direction * (width - 1) * disp_range;
			img_col += direction * (width - 1);
		}

		// Minimum cost of the previous pixel on the path
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// Aggregation starts from the second pixel in the direction
		for (sint32 i = 0; i < height - 1; i++) {
			gray = *img_col;
			uint8 min_cost = UINT8_MAX;
			for (sint32 d = 0; d < disp_range; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
				const uint8  cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));

				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// Reset the minimum cost and cost array of the previous pixel
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

			// Row and column numbers of the current pixel
			current_row += direction;
			current_col -= direction;

			// Next pixel, additional boundary handling is required here
			// When moving along the diagonal, if the image column boundary is reached, the row number continues in the original direction, and the column number jumps to the other boundary
			if (is_forward && current_col == 0 && current_row < height - 1) {
				// Top right to bottom left, reaching the left boundary
				cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				img_col = img_data + (current_row + direction) * width + (width - 1);
				current_col = width - 1;
			}
			else if (!is_forward && current_col == width - 1 && current_row > 0) {
				// Bottom left to top right, reaching the right boundary
				cost_init_col = cost_init + (current_row + direction) * width * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
				img_col = img_data + (current_row + direction) * width;
				current_col = 0;
			}
			else {
				cost_init_col += direction * (width - 1) * disp_range;
				cost_aggr_col += direction * (width - 1) * disp_range;
				img_col += direction * (width - 1);
			}

			// Reassign pixel value
			gray_last = gray;
		}
	}
}

void sgm_util::RemoveSpeckles(float32* disparity_map, const sint32& width, const sint32& height,
	const sint32& diff_insame, const uint32& min_speckle_aera, const float& invalid_val) {
	assert(width > 0 && height > 0);
	if (width < 0 || height < 0) {
		return;
	}

	// Define an array to mark whether the pixel is visited
	std::vector<bool> visited(uint32(width * height), false);
	for (sint32 i = 0; i < height; i++) {
		for (sint32 j = 0; j < width; j++) {
			if (visited[i * width + j] || disparity_map[i * width + j] == invalid_val) {
				// Skip visited pixels and invalid pixels
				continue;
			}

			// Breadth-first search, region tracking
			// Set the disparity of connected components smaller than the threshold to invalid
			std::vector<std::pair<sint32, sint32>> vec;
			vec.emplace_back(i, j);
			visited[i * width + j] = true;
			uint32 cur = 0;
			uint32 next = 0;
			do {
				// Breadth-first search region tracking
				next = vec.size();
				for (uint32 k = cur; k < next; k++) {
					const auto& pixel = vec[k];
					const sint32 row = pixel.first;
					const sint32 col = pixel.second;
					const auto& disp_base = disparity_map[row * width + col];
					// 8-connected neighborhood traversal
					for (int r = -1; r <= 1; r++) {
						for (int c = -1; c <= 1; c++) {
							if (r == 0 && c == 0) {
								continue;
							}
							int rowr = row + r;
							int colc = col + c;
							if (rowr >= 0 && rowr < height && colc >= 0 && colc < width) {
								if (!visited[rowr * width + colc] && abs(disparity_map[rowr * width + colc] - disp_base) <= diff_insame) {
									vec.emplace_back(rowr, colc);
									visited[rowr * width + colc] = true;
								}
							}
						}
					}
				}
				cur = next;
			} while (next < vec.size());

			// Set the disparity of connected components smaller than the threshold to invalid
			if (vec.size() < min_speckle_aera) {
				for (auto& pix : vec) {
					disparity_map[pix.first * width + pix.second] = invalid_val;
				}
			}
		}
	}
}

void sgm_util::MedianFilter(const float32* in, float32* out, const sint32& width, const sint32& height,
	const sint32 wnd_size)
{
	const sint32 radius = wnd_size / 2;
	const sint32 size = wnd_size * wnd_size;

	// Store data within the local window
	std::vector<float32> wnd_data;
	wnd_data.reserve(size);

	for (sint32 i = 0; i < height; i++) {
		for (sint32 j = 0; j < width; j++) {
			wnd_data.clear();

			// Get local window data
			for (sint32 r = -radius; r <= radius; r++) {
				for (sint32 c = -radius; c <= radius; c++) {
					const sint32 row = i + r;
					const sint32 col = j + c;
					if (row >= 0 && row < height && col >= 0 && col < width) {
						wnd_data.push_back(in[row * width + col]);
					}
				}
			}

			// Sort the data
			std::sort(wnd_data.begin(), wnd_data.end());

			// Take the median value
			out[i * width + j] = wnd_data[wnd_data.size() / 2];
		}
	}
}

