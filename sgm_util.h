#pragma once
#include "sgm_types.h"

#ifndef SAFE_DELETE
#define SAFE_DELETE(P) {if(P) delete[](P);(P)=nullptr;}
#endif

namespace sgm_util
{
	//¡¤¡¤¡¤¡¤¡¤¡¤ Census toolkit
	// Census transform

	/**
  * \brief Census transform
  * \param source	Input, image data
  * \param census	Output, census value array
  * \param width	Input, image width
  * \param height	Input, image height
  */
	void CheckCensusTransform(const uint32* census, const sint32& width, const sint32& height);
	void census_transform_5x5(const uint8* source, uint32* census, const sint32& width, const sint32& height);
	void census_transform_9x7(const uint8* source, uint64* census, const sint32& width, const sint32& height);

	// Hamming distance
	uint8 Hamming32(const uint32& x, const uint32& y);
	uint8 Hamming64(const uint64& x, const uint64& y);

	/**
  * \brief Cost aggregation from left to right ¡ú ¡û
  * \param img_data			Input, image data
  * \param width			Input, image width
  * \param height			Input, image height
  * \param min_disparity	Input, minimum disparity
  * \param max_disparity	Input, maximum disparity
  * \param p1				Input, penalty term P1
  * \param p2_init			Input, penalty term P2_Init
  * \param cost_init		Input, initial cost data
  * \param cost_aggr		Output, cost aggregation data
  * \param is_forward		Input, whether it is in the forward direction (forward direction is from left to right, reverse direction is from right to left)
  */
	void CostAggregateLeftRight(const uint8* img_data, const sint32& width, const sint32& height, const sint32& min_disparity, const sint32& max_disparity,
		const sint32& p1, const sint32& p2_init, const uint8* cost_init, uint8* cost_aggr, bool is_forward = true);

	/**
  * \brief Cost aggregation from top to bottom ¡ý ¡ü
  * \param img_data			Input, image data
  * \param width			Input, image width
  * \param height			Input, image height
  * \param min_disparity	Input, minimum disparity
  * \param max_disparity	Input, maximum disparity
  * \param p1				Input, penalty term P1
  * \param p2_init			Input, penalty term P2_Init
  * \param cost_init		Input, initial cost data
  * \param cost_aggr		Output, cost aggregation data
  * \param is_forward		Input, whether it is in the forward direction (forward direction is from top to bottom, reverse direction is from bottom to top)
  */
	void CostAggregateUpDown(const uint8* img_data, const sint32& width, const sint32& height, const sint32& min_disparity, const sint32& max_disparity,
		const sint32& p1, const sint32& p2_init, const uint8* cost_init, uint8* cost_aggr, bool is_forward = true);

	/**
  * \brief Cost aggregation along diagonal 1 (top-left <-> bottom-right) ¨K ¨I
  * \param img_data			Input, image data
  * \param width			Input, image width
  * \param height			Input, image height
  * \param min_disparity	Input, minimum disparity
  * \param max_disparity	Input, maximum disparity
  * \param p1				Input, penalty term P1
  * \param p2_init			Input, penalty term P2_Init
  * \param cost_init		Input, initial cost data
  * \param cost_aggr		Output, cost aggregation data
  * \param is_forward		Input, whether it is in the forward direction (forward direction is from top-left to bottom-right, reverse direction is from bottom-right to top-left)
  */
	void CostAggregateDagonal_1(const uint8* img_data, const sint32& width, const sint32& height, const sint32& min_disparity, const sint32& max_disparity,
		const sint32& p1, const sint32& p2_init, const uint8* cost_init, uint8* cost_aggr, bool is_forward = true);

	/**
  * \brief Cost aggregation along diagonal 2 (top-right <-> bottom-left) ¨L ¨J
  * \param img_data			Input, image data
  * \param width			Input, image width
  * \param height			Input, image height
  * \param min_disparity	Input, minimum disparity
  * \param max_disparity	Input, maximum disparity
  * \param p1				Input, penalty term P1
  * \param p2_init			Input, penalty term P2_Init
  * \param cost_init		Input, initial cost data
  * \param cost_aggr		Output, cost aggregation data
  * \param is_forward		Input, whether it is in the forward direction (forward direction is from top to bottom, reverse direction is from bottom to top)
  */
	void CostAggregateDagonal_2(const uint8* img_data, const sint32& width, const sint32& height, const sint32& min_disparity, const sint32& max_disparity,
		const sint32& p1, const sint32& p2_init, const uint8* cost_init, uint8* cost_aggr, bool is_forward = true);


	/**
  * \brief Median filter
  * \param in				Input, source data
  * \param out				Output, target data
  * \param width			Input, width
  * \param height			Input, height
  * \param wnd_size			Input, window width
  */
	void MedianFilter(const float32* in, float32* out, const sint32& width, const sint32& height, const sint32 wnd_size);


	/**
  * \brief Remove small speckle regions
  * \param disparity_map		Input, disparity map
  * \param width				Input, width
  * \param height				Input, height
  * \param diff_insame			Input, local pixel difference within the same region
  * \param min_speckle_aera		Input, minimum speckle area
  * \param invalid_val			Input, invalid value
  */
	void RemoveSpeckles(float32* disparity_map, const sint32& width, const sint32& height, const sint32& diff_insame, const uint32& min_speckle_aera, const float32& invalid_val);
}