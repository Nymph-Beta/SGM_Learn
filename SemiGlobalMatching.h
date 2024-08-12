#pragma once

#include "sgm_types.h"
#include <vector>

/**
 * \brief SemiGlobalMatching class (General implementation of Semi-Global Matching)
 */
class SemiGlobalMatching
{
public:
	SemiGlobalMatching();
	~SemiGlobalMatching();


	/** \brief Census window size types  */
	enum CensusSize {
		Census5x5 = 0,
		Census9x7
	};

	/** \brief SGM parameters structure */
	struct SGMOption {
		uint8	num_paths;			// Number of aggregation paths, 4 and 8
		sint32  min_disparity;		// Minimum disparity
		sint32	max_disparity;		// Maximum disparity

		CensusSize census_size;		// Census window size

		bool	is_check_unique;	// Whether to check uniqueness
		float32	uniqueness_ratio;	// Uniqueness constraint threshold (minimum cost - second minimum cost) / minimum cost > threshold is considered valid pixel

		bool	is_check_lr;		// Whether to check left-right consistency
		float32	lrcheck_thres;		// Left-right consistency constraint threshold

		bool	is_remove_speckles;	// Whether to remove small connected regions
		int		min_speckle_aera;	// Minimum connected region area (number of pixels)

		bool	is_fill_holes;		// Whether to fill disparity holes

		// P1,P2 
		// P2 = P2_init / (Ip-Iq)
		sint32  p1;				// Penalty parameter P1
		sint32  p2_init;		// Penalty parameter P2

		SGMOption() : num_paths(8), min_disparity(0), max_disparity(64), census_size(Census5x5),
			is_check_unique(true), uniqueness_ratio(0.95f),
			is_check_lr(true), lrcheck_thres(1.0f),
			is_remove_speckles(true), min_speckle_aera(20),
			is_fill_holes(true),
			p1(10), p2_init(150) { }
	};
public:
	/**
  * \brief Initialize the class, pre-allocate memory, set parameters, etc.
  * \param width		Input, width of the rectified stereo image
  * \param height		Input, height of the rectified stereo image
  * \param option		Input, SemiGlobalMatching parameters
  */
	bool Initialize(const sint32& width, const sint32& height, const SGMOption& option);

	/**
  * \brief Perform matching
  * \param img_left		Input, pointer to the left image data
  * \param img_right	Input, pointer to the right image data
  * \param disp_left	Output, pointer to the disparity map of the left image, pre-allocated with the same size as the images
  */
	bool Match(const uint8* img_left, const uint8* img_right, float32* disp_left);

	/**
  * \brief Reset
  * \param width		Input, width of the rectified stereo image
  * \param height		Input, height of the rectified stereo image
  * \param option		Input, SemiGlobalMatching parameters
  */
	bool Reset(const uint32& width, const uint32& height, const SGMOption& option);

private:

	/** \brief Census transform */
	void CensusTransform() const;

	/** \brief Cost computation */
	void ComputeCost() const;

	/** \brief Cost aggregation */
	void CostAggregation() const;

	/** \brief Disparity computation */
	void ComputeDisparity() const;

	/** \brief Disparity computation (right image) */
	void ComputeDisparityRight() const;

	/** \brief Left-right consistency check */
	void LRCheck();

	/** \brief Fill holes in disparity map */
	void FillHolesInDispMap();

	/** \brief Memory release */
	void Release();

private:
	/** \brief SGM parameters */
	SGMOption option_;

	/** \brief Image width */
	sint32 width_;

	/** \brief Image height */
	sint32 height_;

	/** \brief Left image data */
	const uint8* img_left_;

	/** \brief Right image data */
	const uint8* img_right_;

	/** \brief Left image census values */
	uint32* census_left_;

	/** \brief Right image census values */
	uint32 * census_right_;

	/** \brief Initial matching cost */
	uint8* cost_init_;

	/** \brief Aggregated matching cost */
	uint16* cost_aggr_;

	// ¨K ¡ý ¨L   5  3  7
	// ¡ú    ¡û	 1    2
	// ¨J ¡ü ¨I   8  4  6
	/** \brief Aggregated matching cost - direction 1 */
	uint8* cost_aggr_1_;
	/** \brief Aggregated matching cost - direction 2 */
	uint8* cost_aggr_2_;
	/** \brief Aggregated matching cost - direction 3 */
	uint8* cost_aggr_3_;
	/** \brief Aggregated matching cost - direction 4 */
	uint8* cost_aggr_4_;
	/** \brief Aggregated matching cost - direction 5 */
	uint8* cost_aggr_5_;
	/** \brief Aggregated matching cost - direction 6 */
	uint8* cost_aggr_6_;
	/** \brief Aggregated matching cost - direction 7 */
	uint8* cost_aggr_7_;
	/** \brief Aggregated matching cost - direction 8 */
	uint8* cost_aggr_8_;

	/** \brief Left image disparity map */
	float32* disp_left_;
	/** \brief Right image disparity map */
	float32* disp_right_;

	/** \brief Initialization flag */
	bool is_initialized_;

	/** \brief Occlusion pixel set */
	std::vector<std::pair<int, int>> occlusions_;
	/** \brief Mismatch pixel set */
	std::vector<std::pair<int, int>> mismatches_;
};
