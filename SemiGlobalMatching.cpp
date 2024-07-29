#include "stdafx.h"
#include "SemiGlobalMatching.h"
#include "sgm_util.h"
#include <algorithm>
#include <vector>
#include <cassert>
#include <chrono>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono;

SemiGlobalMatching::SemiGlobalMatching() : width_(0), height_(0), img_left_(nullptr), img_right_(nullptr),
                                           census_left_(nullptr), census_right_(nullptr),
                                           cost_init_(nullptr), cost_aggr_(nullptr),
                                           cost_aggr_1_(nullptr), cost_aggr_2_(nullptr),
                                           cost_aggr_3_(nullptr), cost_aggr_4_(nullptr),
                                           cost_aggr_5_(nullptr), cost_aggr_6_(nullptr),
                                           cost_aggr_7_(nullptr), cost_aggr_8_(nullptr),
                                           disp_left_(nullptr), disp_right_(nullptr),
                                           is_initialized_(false)
{
}


SemiGlobalMatching::~SemiGlobalMatching()
{
    Release();
    is_initialized_ = false;
}


bool SemiGlobalMatching::Initialize(const sint32& width, const sint32& height, const SGMOption& option)
{
    // ··· Assign values

    // Image dimensions
    width_ = width;
    height_ = height;

    //std::cout << "width_: " << width_ << std::endl;
    //std::cout << "height_: " << height_ << std::endl;

    // SGM options
    option_ = option;

    if (width == 0 || height == 0) {
        return false;
    }

    //··· Allocate memory

    // Census values (left and right images)
    census_left_ = new uint32[width * height]();
    census_right_ = new uint32[width * height]();

    // Matching costs (initial and aggregated)
    const sint32 disp_range = option.max_disparity - option.min_disparity;
    if (disp_range <= 0) {
        return false;
    }
    cost_init_ = new uint8[width * height * disp_range]();
    cost_aggr_ = new uint16[width * height * disp_range]();

    // Disparity maps
    disp_left_ = new float32[width * height]();

    is_initialized_ = census_left_ && census_right_ && cost_init_ && cost_aggr_ && disp_left_;

    return is_initialized_;
}

bool SemiGlobalMatching::Match(const uint8* img_left, const uint8* img_right, float32* disp_left)
{
    if (!is_initialized_) {
        return false;
    }
    if (img_left == nullptr || img_right == nullptr) {
        return false;
    }

    img_left_ = img_left;
    img_right_ = img_right;

    // Census transform
    CensusTransform();

    // Cost computation
    ComputeCost();

    // Cost aggregation
    // CostAggregation();

    // Disparity computation
    ComputeDisparity();

    // Output disparity map
    memcpy(disp_left, disp_left_, width_ * height_ * sizeof(float32));

    return true;
}


void SemiGlobalMatching::Release()
{
    // Release memory
    SAFE_DELETE(census_left_);
    SAFE_DELETE(census_right_);
    SAFE_DELETE(cost_init_);
    SAFE_DELETE(cost_aggr_);
    SAFE_DELETE(cost_aggr_1_);
    SAFE_DELETE(cost_aggr_2_);
    SAFE_DELETE(cost_aggr_3_);
    SAFE_DELETE(cost_aggr_4_);
    SAFE_DELETE(cost_aggr_5_);
    SAFE_DELETE(cost_aggr_6_);
    SAFE_DELETE(cost_aggr_7_);
    SAFE_DELETE(cost_aggr_8_);
    SAFE_DELETE(disp_left_);
    SAFE_DELETE(disp_right_);
}

bool SemiGlobalMatching::Reset(const uint32& width, const uint32& height, const SGMOption& option)
{
    // Release memory
    Release();

    // Reset initialization flag
    is_initialized_ = false;

    // Initialize
    return Initialize(width, height, option);
}

void sgm_util::CheckCensusTransform(const uint32* census, const sint32& width, const sint32& height) {
    for (sint32 i = 2; i < height - 2; i++) {
        for (sint32 j = 2; j < width - 2; j++) {
            std::cout << "Census value at (" << i << ", " << j << "): " << census[i * width + j] << std::endl;
        }
    }
}

void SemiGlobalMatching::CensusTransform() const
{
    //std::cout << "img_left_: " << img_left_ << std::endl;
    //std::cout << "img_right_: " << img_right_ << std::endl;


    // Census transform for left and right images
    sgm_util::census_transform_5x5(img_left_, static_cast<uint32*>(census_left_), width_, height_);
    sgm_util::census_transform_5x5(img_right_, static_cast<uint32*>(census_right_), width_, height_);
    // sgm_util::CheckCensusTransform(census_left_, width_, height_);
    // sgm_util::CheckCensusTransform(census_right_, width_, height_);
}



void SemiGlobalMatching::ComputeCost() const
{
    std::cout << "census_left: " << census_left_ << std::endl;
    std::cout << "census_right: " << census_right_ << std::endl;


    const sint32& min_disparity = option_.min_disparity;
    const sint32& max_disparity = option_.max_disparity;
    const sint32 disp_range = max_disparity - min_disparity;
    if (disp_range <= 0) {
        return;
    }

    // Compute costs based on Hamming distance
    for (sint32 i = 0; i < height_; i++) {
        for (sint32 j = 0; j < width_; j++) {

            // const uint32 census_val_l = census_left_[i * width_ + j];
            // std::cout << "census_val_l: " << census_val_l << std::endl;

            // Compute cost for each disparity
            for (sint32 d = min_disparity; d < max_disparity; d++) {
                auto& cost = cost_init_[i * width_ * disp_range + j * disp_range + (d - min_disparity)];
                if (j - d < 0 || j - d >= width_) {
                    cost = UINT8_MAX / 2;
                    continue;
                }

                // Left image corresponding point census value
                 const uint32 census_val_l = static_cast<uint32*>(census_left_)[i * width_ + j];
                // Right image corresponding point census value
                const uint32 census_val_r = static_cast<uint32*>(census_right_)[i * width_ + j - d];

                //std::cout << "At pixel (" << i << ", " << j << "), disparity " << d << ":" << std::endl;
                //std::cout << "census_val_l: " << census_val_l << std::endl;
                //std::cout << "census_val_r: " << census_val_r << std::endl;


                // Compute matching cost
                cost = sgm_util::Hamming32(census_val_l, census_val_r);
                //std::cout << "cost: " << cost << std::endl;
                //std::cout << "cost: " << static_cast<int>(cost) << std::endl;
            }
        }
    }
}

void SemiGlobalMatching::ComputeDisparity() const
{
    // Minimum and maximum disparity
    const sint32& min_disparity = option_.min_disparity;
    const sint32& max_disparity = option_.max_disparity;
    const sint32 disp_range = max_disparity - min_disparity;
    if (disp_range <= 0) {
        return;
    }

    // 未实现聚合步骤，暂用初始代价值来代替
    auto cost_ptr = cost_init_;

    // Compute optimal disparity for each pixel
    for (sint32 i = 0; i < height_; i++) {
        for (sint32 j = 0; j < width_; j++) {

            uint16 min_cost = UINT16_MAX;
            uint16 max_cost = 0;
            sint32 best_disparity = 0;

            // Find minimum cost and corresponding disparity
            for (sint32 d = min_disparity; d < max_disparity; d++) {
                const sint32 d_idx = d - min_disparity;
                const auto& cost = cost_ptr[i * width_ * disp_range + j * disp_range + d_idx];
                if (min_cost > cost) {
                    min_cost = cost;
                    best_disparity = d;
                }
                max_cost = std::max(max_cost, static_cast<uint16>(cost));
            }

            // Set disparity to the minimum cost disparity
            if (max_cost != min_cost) {
                disp_left_[i * width_ + j] = static_cast<float>(best_disparity);
            }
            else {
                // If all costs are equal, mark pixel as invalid
                disp_left_[i * width_ + j] = Invalid_Float;
            }
        }
    }
}

/**
 * \brief Main function
 * \param argv 3 arguments: left image path, right image path, output directory path
 * \param argc Number of arguments
 * \return 0 if success, non-zero otherwise
 */
int main(int argc, char** argv) {
    if (argc < 4) {  // Ensure there are three command-line arguments (program name + three paths)
        std::cout << "Usage: " << argv[0] << " <left_image> <right_image> <output_directory>" << std::endl;
        return 0;
    }

    // Read image paths
    std::string path_left = argv[1];
    std::string path_right = argv[2];
    std::string output_directory = argv[3];

    // Use OpenCV functions to read images
    cv::Mat img_left = cv::imread(path_left, cv::IMREAD_GRAYSCALE);
    cv::Mat img_right = cv::imread(path_right, cv::IMREAD_GRAYSCALE);

    // Print file paths for debugging
    std::cout << "Left image path: " << path_left << std::endl;
    std::cout << "Right image path: " << path_right << std::endl;

    //std::cout << "img_left:\n" << img_left << std::endl;
    //std::cout << "img_right:\n" << img_right << std::endl;

    // Check if images were successfully read
    if (img_left.data == nullptr || img_right.data == nullptr) {
        std::cout << "Failed to read images!！" << std::endl;
        return -1;
    }
    if (img_left.rows != img_right.rows || img_left.cols != img_right.cols) {
        std::cout << "Left and right images have different dimensions!" << std::endl;
        return -1;
    }

    // SGM匹配
    const uint32 width = static_cast<uint32>(img_left.cols);
    const uint32 height = static_cast<uint32>(img_right.rows);

    std::cout << "width_: " << width << std::endl;
    std::cout << "height_: " << height << std::endl;

    SemiGlobalMatching::SGMOption sgm_option;
    sgm_option.num_paths = 8;
    sgm_option.min_disparity = 0;
    sgm_option.max_disparity = 64;
    sgm_option.p1 = 10;
    sgm_option.p2_init = 1500;

    SemiGlobalMatching sgm;

    // Initialize
    if (!sgm.Initialize(width, height, sgm_option)) {
        std::cout << "SGM initialization failed!" << std::endl;
        return -2;
    }

    // Perform matching
    auto disparity = new float32[width * height]();
    if (!sgm.Match(img_left.data, img_right.data, disparity)) {
        std::cout << "SGM matching failed!" << std::endl;
        return -2;
    }

    // Display disparity map
    cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
    for (uint32 i = 0; i < height; i++) {
        for (uint32 j = 0; j < width; j++) {
            const float32 disp = disparity[i * width + j];
            if (disp == Invalid_Float) {
                disp_mat.data[i * width + j] = 0;
            }
            else {
                disp_mat.data[i * width + j] = 2 * static_cast<uchar>(disp);
            }
        }
    }


    // Normalize disparity map
    cv::Mat disp_norm;
    cv::normalize(disp_mat, disp_norm, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Save disparity map
    std::string output_path = output_directory + "/disparity.png";
    if (!cv::imwrite(output_path, disp_mat)) {
        std::cout << "Failed to save disparity image to " << output_path << std::endl;
        delete[] disparity;
        return -1;
    }

    cv::imshow("Disparity Map", disp_norm);
    //cv::imshow("视差图", disp_mat);
    cv::waitKey(0);

    delete[] disparity;
    disparity = nullptr;

    return 0;
}

