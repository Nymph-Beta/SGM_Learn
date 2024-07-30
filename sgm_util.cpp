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

	// 打印census值
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
	const sint32 disp_range = max_disparity - min_disparity ;	

	const auto& P1 = p1;
	const auto& P2 = p2_init;

	const sint32 direction = is_forward ? 1 : -1;

	for (sint32 i = 0u; i < height; i++) {
		//auto* cost_aggr_row = cost_aggr + i * width * disp_range;
		auto cost_init_row = (is_forward) ? cost_init + i * width * disp_range : cost_init + i * width * disp_range + (width - 1) * disp_range;
		auto cost_aggr_row = (is_forward) ? cost_aggr + i * width * disp_range : cost_aggr + i * width * disp_range + (width - 1) * disp_range;
		auto img_row = (is_forward) ? img_data + i * width : img_data + i * width + (width - 1);

		uint8 gray = *img_row;
		uint8 gray_last = *img_row;

		std::vector<uint8>cost_last_path(disp_range + 2, UINT8_MAX);

		memcpy(cost_aggr_row, cost_init_row, disp_range * sizeof(uint8));
		//memcpy(cost_last_path.data() + 1, cost_aggr_row, disp_range * sizeof(uint8));
		memcpy(&cost_last_path[1], cost_aggr_row, disp_range * sizeof(uint8));
		cost_init_row += direction* disp_range;
		cost_aggr_row += direction* disp_range;
		img_row += direction;

		uint8 mincost_last_path = UINT8_MAX;
		for(auto cost : cost_last_path)
			mincost_last_path = std::min(mincost_last_path, cost);

		for (sint32 j = 0; j < width - 1; j++) {
			gray = *img_row;
			uint8 min_cost = UINT8_MAX;
			const sint32 disp_offset = j * disp_range;

			// Compute the cost of the first disparity
			//cost_aggr_row[0] = std::min(cost_aggr_row[0], static_cast<uint8>(cost_last_path[0] + P1));
			//cost_aggr_row[0] = std::min(cost_aggr_row[0], static_cast<uint8>(cost_last_path[1] + P2));

			// Compute the cost of the remaining disparities
			for (sint32 d = 0; d < disp_range; d++) {
				//const sint32 cost_min = std::min(cost_last_path[d + 1], cost_last_path[d + 2]);
				//cost_aggr_row[d] = std::min(cost_aggr_row[d], static_cast<uint8>(cost_min + P1));
				//cost_aggr_row[d] = std::min(cost_aggr_row[d], static_cast<uint8>(cost_last_path[d] + P2));
				//cost_aggr_row[d] = std::min(cost_aggr_row[d], static_cast<uint8>(cost_last_path[d + 2] + P2));
				const uint8 cost = cost_init_row[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2 / (abs(gray - gray_last) + 1));

				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_row[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// Update the last path cost
			//cost_last_path[0] = cost_aggr_row[0];
			//for (sint32 d = 1; d < disp_range + 1; d++) {
			//	cost_last_path[d] = cost_aggr_row[d];
			//}

			mincost_last_path = min_cost;
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
		//auto* cost_aggr_row = cost_aggr + i * width * disp_range;
		//auto cost_init_col = (is_forward) ? cost_init + j * width * disp_range : cost_init + j * width * disp_range + (width - 1) * disp_range;
		//auto cost_aggr_col = (is_forward) ? cost_aggr + j * width * disp_range : cost_aggr + j * width * disp_range + (width - 1) * disp_range;
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		//auto img_col = (is_forward) ? img_data + j * width : img_data + j * width + (width - 1);
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
				//const sint32 cost_min = std::min(cost_last_path[d + 1], cost_last_path[d + 2]);
				//cost_aggr_row[d] = std::min(cost_aggr_row[d], static_cast<uint8>(cost_min + P1));
				//cost_aggr_row[d] = std::min(cost_aggr_row[d], static_cast<uint8>(cost_last_path[d] + P2));
				//cost_aggr_row[d] = std::min(cost_aggr_row[d], static_cast<uint8>(cost_last_path[d + 2] + P2));
				const uint8 cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2 / (abs(gray - gray_last) + 1));

				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// Update the last path cost
			//cost_last_path[0] = cost_aggr_row[0];
			//for (sint32 d = 1; d < disp_range + 1; d++) {
			//	cost_last_path[d] = cost_aggr_row[d];
			//}

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

	// 视差范围
	const sint32 disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// 正向(左上->右下) ：is_forward = true ; direction = 1
	// 反向(右下->左上) ：is_forward = false; direction = -1;
	const sint32 direction = is_forward ? 1 : -1;

	// 聚合

	// 存储当前的行列号，判断是否到达影像边界
	sint32 current_row = 0;
	sint32 current_col = 0;

	for (sint32 j = 0; j < width; j++) {
		// 路径头为每一列的首(尾,dir=-1)行像素
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		// 路径上上个像素的代价数组，多两个元素是为了避免边界溢出（首尾各多一个）
		std::vector<uint8> cost_last_path(disp_range + 2, UINT8_MAX);

		// 初始化：第一个像素的聚合代价值等于初始代价值
		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(uint8));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

		// 路径上当前灰度值和上一个灰度值
		uint8 gray = *img_col;
		uint8 gray_last = *img_col;

		// 对角线路径上的下一个像素，中间间隔width+1个像素
		// 这里要多一个边界处理
		// 沿对角线前进的时候会碰到影像列边界，策略是行号继续按原方向前进，列号到跳到另一边界
		current_row = is_forward ? 0 : height - 1;
		current_col = j;
		if (is_forward && current_col == width - 1 && current_row < height - 1) {
			// 左上->右下，碰右边界
			cost_init_col = cost_init + (current_row + direction) * width * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
			img_col = img_data + (current_row + direction) * width;
			current_col = 0;
		}
		else if (!is_forward && current_col == 0 && current_row > 0) {
			// 右下->左上，碰左边界
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

		// 路径上上个像素的最小代价值
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// 自方向上第2个像素开始按顺序聚合
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

			// 重置上个像素的最小代价值和代价数组
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

			// 当前像素的行列号
			current_row += direction;
			current_col += direction;

			// 下一个像素,这里要多一个边界处理
			// 这里要多一个边界处理
			// 沿对角线前进的时候会碰到影像列边界，策略是行号继续按原方向前进，列号到跳到另一边界
			if (is_forward && current_col == width - 1 && current_row < height - 1) {
				// 左上->右下，碰右边界
				cost_init_col = cost_init + (current_row + direction) * width * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
				img_col = img_data + (current_row + direction) * width;
				current_col = 0;
			}
			else if (!is_forward && current_col == 0 && current_row > 0) {
				// 右下->左上，碰左边界
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

			// 像素值重新赋值
			gray_last = gray;
		}
	}
}

void sgm_util::CostAggregateDagonal_2(const uint8* img_data, const sint32& width, const sint32& height,
	const sint32& min_disparity, const sint32& max_disparity, const sint32& p1, const sint32& p2_init,
	const uint8* cost_init, uint8* cost_aggr, bool is_forward)
{
	assert(width > 1 && height > 1 && max_disparity > min_disparity);

	// 视差范围
	const sint32 disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// 正向(右上->左下) ：is_forward = true ; direction = 1
	// 反向(左下->右上) ：is_forward = false; direction = -1;
	const sint32 direction = is_forward ? 1 : -1;

	// 聚合

	// 存储当前的行列号，判断是否到达影像边界
	sint32 current_row = 0;
	sint32 current_col = 0;

	for (sint32 j = 0; j < width; j++) {
		// 路径头为每一列的首(尾,dir=-1)行像素
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		// 路径上上个像素的代价数组，多两个元素是为了避免边界溢出（首尾各多一个）
		std::vector<uint8> cost_last_path(disp_range + 2, UINT8_MAX);

		// 初始化：第一个像素的聚合代价值等于初始代价值
		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(uint8));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

		// 路径上当前灰度值和上一个灰度值
		uint8 gray = *img_col;
		uint8 gray_last = *img_col;

		// 对角线路径上的下一个像素，中间间隔width-1个像素
		// 这里要多一个边界处理
		// 沿对角线前进的时候会碰到影像列边界，策略是行号继续按原方向前进，列号到跳到另一边界
		current_row = is_forward ? 0 : height - 1;
		current_col = j;
		if (is_forward && current_col == 0 && current_row < height - 1) {
			// 右上->左下，碰左边界
			cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			img_col = img_data + (current_row + direction) * width + (width - 1);
			current_col = width - 1;
		}
		else if (!is_forward && current_col == width - 1 && current_row > 0) {
			// 左下->右上，碰右边界
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

		// 路径上上个像素的最小代价值
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// 自路径上第2个像素开始按顺序聚合
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

			// 重置上个像素的最小代价值和代价数组
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

			// 当前像素的行列号
			current_row += direction;
			current_col -= direction;

			// 下一个像素,这里要多一个边界处理
			// 这里要多一个边界处理
			// 沿对角线前进的时候会碰到影像列边界，策略是行号继续按原方向前进，列号到跳到另一边界
			if (is_forward && current_col == 0 && current_row < height - 1) {
				// 右上->左下，碰左边界
				cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				img_col = img_data + (current_row + direction) * width + (width - 1);
				current_col = width - 1;
			}
			else if (!is_forward && current_col == width - 1 && current_row > 0) {
				// 左下->右上，碰右边界
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

			// 像素值重新赋值
			gray_last = gray;
		}
	}
}