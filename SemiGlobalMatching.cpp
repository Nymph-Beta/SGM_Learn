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


bool SemiGlobalMatching::Initialize(const sint32& width, const sint32& height, const SGMOption& option){
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

    const sint32 size = width * height * disp_range;

    cost_init_ = new uint8[width * height * disp_range]();
    cost_aggr_ = new uint16[width * height * disp_range]();

    cost_aggr_1_ = new uint8[size]();
    cost_aggr_2_ = new uint8[size]();
    cost_aggr_3_ = new uint8[size]();
    cost_aggr_4_ = new uint8[size]();
    cost_aggr_5_ = new uint8[size]();
    cost_aggr_6_ = new uint8[size]();
    cost_aggr_7_ = new uint8[size]();
    cost_aggr_8_ = new uint8[size]();

    // Disparity maps
    disp_left_ = new float32[width * height]();
    disp_right_ = new float32[width * height]();

    is_initialized_ = census_left_ && census_right_ && cost_init_ && cost_aggr_ && disp_left_;

    return is_initialized_;
}

bool SemiGlobalMatching::Match(const uint8* img_left, const uint8* img_right, float32* disp_left){
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
    CostAggregation();

    // Disparity computation
    ComputeDisparity();

    // 左右一致性检查
    if (option_.is_check_lr) {
        // 视差计算（右影像）
        ComputeDisparityRight();
        // 一致性检查
        LRCheck();
    }

    // 移除小连通区
    if (option_.is_remove_speckles) {
        sgm_util::RemoveSpeckles(disp_left_, width_, height_, 1, option_.min_speckle_aera, Invalid_Float);
    }

    // 视差填充
    if (option_.is_fill_holes) {
        FillHolesInDispMap();
    }

    // 中值滤波
    sgm_util::MedianFilter(disp_left_, disp_left_, width_, height_, 3);

    // Output disparity map
    memcpy(disp_left, disp_left_, width_ * height_ * sizeof(float32));

    return true;
}


void SemiGlobalMatching::Release(){
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

bool SemiGlobalMatching::Reset(const uint32& width, const uint32& height, const SGMOption& option){
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

void SemiGlobalMatching::CensusTransform() const{
    //std::cout << "img_left_: " << img_left_ << std::endl;
    //std::cout << "img_right_: " << img_right_ << std::endl;


    // Census transform for left and right images
    sgm_util::census_transform_5x5(img_left_, static_cast<uint32*>(census_left_), width_, height_);
    sgm_util::census_transform_5x5(img_right_, static_cast<uint32*>(census_right_), width_, height_);
    // sgm_util::CheckCensusTransform(census_left_, width_, height_);
    // sgm_util::CheckCensusTransform(census_right_, width_, height_);
}

void SemiGlobalMatching::CostAggregation() const {
    // 路径聚合
    // 1、左->右/右->左
    // 2、上->下/下->上
    // 3、左上->右下/右下->左上
    // 4、右上->左上/左下->右上
    //
    // ↘ ↓ ↙   5  3  7
    // →    ←	 1    2
    // ↗ ↑ ↖   8  4  6
    //

    std::cout << " CostAggregation strat " << std::endl;
    const auto& min_disparity = option_.min_disparity;
    const auto& max_disparity = option_.max_disparity;
    assert(max_disparity > min_disparity);

    const sint32 size = width_ * height_ * (max_disparity - min_disparity);
    if (size <= 0) {
        return;
    }

    const auto& P1 = option_.p1;
    const auto& P2 = option_.p2_init;


    std::cout << " CostAggregateLeftRight strat " << std::endl;
    sgm_util::CostAggregateLeftRight(img_left_, width_, height_, min_disparity, max_disparity, P1, P2, cost_init_, cost_aggr_1_, true);
    sgm_util::CostAggregateLeftRight(img_left_, width_, height_, min_disparity, max_disparity, P1, P2, cost_init_, cost_aggr_2_, false);

    std::cout << " CostAggregateUpDown strat " << std::endl;
    sgm_util::CostAggregateUpDown(img_left_, width_, height_, min_disparity, max_disparity, P1, P2, cost_init_, cost_aggr_3_, true);
    sgm_util::CostAggregateUpDown(img_left_, width_, height_, min_disparity, max_disparity, P1, P2, cost_init_, cost_aggr_4_, false);

    std::cout << " CostAggregateDagonal strat " << std::endl;
    // 对角线1聚合
    sgm_util::CostAggregateDagonal_1(img_left_, width_, height_, min_disparity, max_disparity, P1, P2, cost_init_, cost_aggr_5_, true);
    sgm_util::CostAggregateDagonal_1(img_left_, width_, height_, min_disparity, max_disparity, P1, P2, cost_init_, cost_aggr_6_, false);
    // 对角线2聚合
    sgm_util::CostAggregateDagonal_2(img_left_, width_, height_, min_disparity, max_disparity, P1, P2, cost_init_, cost_aggr_7_, true);
    sgm_util::CostAggregateDagonal_2(img_left_, width_, height_, min_disparity, max_disparity, P1, P2, cost_init_, cost_aggr_8_, false);

    for (sint32 i = 0; i < size; i++) {
        //std::cout << "cost_aggr_1_[i]: " << cost_aggr_1_[i] << std::endl;
        //std::cout << "cost_aggr_2_[i]: " << cost_aggr_2_[i] << std::endl;
        //std::cout << "cost_aggr_3_[i]: " << cost_aggr_3_[i] << std::endl;
        //std::cout << "cost_aggr_4_[i]: " << cost_aggr_4_[i] << std::endl;
        cost_aggr_[i] = cost_aggr_1_[i] + cost_aggr_2_[i] + cost_aggr_3_[i] + cost_aggr_4_[i];
        if (option_.num_paths == 8) {
            cost_aggr_[i] += cost_aggr_5_[i] + cost_aggr_6_[i] + cost_aggr_7_[i] + cost_aggr_8_[i];
        }
    }

}

void SemiGlobalMatching::ComputeCost() const{
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

void SemiGlobalMatching::ComputeDisparity() const{

    std::cout << " ComputeDisparity strat " << std::endl;

    // Minimum and maximum disparity
    const sint32& min_disparity = option_.min_disparity;
    const sint32& max_disparity = option_.max_disparity;
    const sint32 disp_range = max_disparity - min_disparity;
    if (disp_range <= 0) {
        return;
    }

    // 未实现聚合步骤，暂用初始代价值来代替
    //auto cost_ptr = cost_init_;
    const auto cost_ptr = cost_aggr_;


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

void SemiGlobalMatching::ComputeDisparityRight() const {
    const sint32& min_disparity = option_.min_disparity;
    const sint32& max_disparity = option_.max_disparity;
    const sint32 disp_range = max_disparity - min_disparity;
    if (disp_range <= 0) {
        return;
    }

    const auto disparity = disp_right_;
    const auto cost_ptr = cost_aggr_;

    const sint32 width = width_;
    const sint32 height = height_;
    const bool is_check_unique = option_.is_check_unique;
    const float32 uniqueness = option_.uniqueness_ratio;

    std::vector<uint16> cost_local(disp_range);

    // ---逐像素计算最优视差
    // 通过左影像的代价，获取右影像的代价
    // 右cost(xr,yr,d) = 左cost(xr+d,yl,d)
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            uint16 min_cost = UINT16_MAX;
            uint16 sec_min_cost = UINT16_MAX;
            sint32 best_disparity = 0;

            // ---统计候选视差下的代价值
            for (sint32 d = min_disparity; d < max_disparity; d++) {
                const sint32 d_idx = d - min_disparity;
                const sint32 col_left = j + d;
                if (col_left >= 0 && col_left < width) {
                    const auto& cost = cost_local[d_idx] = cost_ptr[i * width * disp_range + col_left * disp_range + d_idx];
                    if (min_cost > cost) {
                        min_cost = cost;
                        best_disparity = d;
                    }
                }
                else {
                    cost_local[d_idx] = UINT16_MAX;
                }
            }

            if (is_check_unique) {
                // 再遍历一次，输出次最小代价值
                for (sint32 d = min_disparity; d < max_disparity; d++) {
                    if (d == best_disparity) {
                        // 跳过最小代价值
                        continue;
                    }
                    const auto& cost = cost_local[d - min_disparity];
                    sec_min_cost = std::min(sec_min_cost, cost);
                }

                // 判断唯一性约束
                // 若(min-sec)/min < min*(1-uniquness)，则为无效估计
                if (sec_min_cost - min_cost <= static_cast<uint16>(min_cost * (1 - uniqueness))) {
                    disparity[i * width + j] = Invalid_Float;
                    continue;
                }
            }

            // ---子像素拟合
            if (best_disparity == min_disparity || best_disparity == max_disparity - 1) {

                //if (disparity == nullptr) {
                //    std::cerr << "Error: disparity pointer is null!" << std::endl;
                //    return;
                //}


                disparity[i * width + j] = Invalid_Float;
                continue;
            }

            // 最优视差前一个视差的代价值cost_1，后一个视差的代价值cost_2
            const sint32 idx_1 = best_disparity - 1 - min_disparity;
            const sint32 idx_2 = best_disparity + 1 - min_disparity;
            const uint16 cost_1 = cost_local[idx_1];
            const uint16 cost_2 = cost_local[idx_2];
            // 解一元二次曲线极值
            const uint16 denom = std::max(1, cost_1 + cost_2 - 2 * min_cost);
            disparity[i * width + j] = static_cast<float32>(best_disparity) + static_cast<float32>(cost_1 - cost_2) / (denom * 2.0f);
        }
    }
}

void SemiGlobalMatching::LRCheck() {
    const sint32 width = width_;
    const sint32 height = height_;

    const float32& threshold = option_.lrcheck_thres;

    // 遮挡区像素和误匹配区像素
    auto& occlusions = occlusions_;
    auto& mismatches = mismatches_;
    occlusions.clear();
    mismatches.clear();

    // ---左右一致性检查
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            // 左影像视差值
            auto& disp = disp_left_[i * width + j];
            if (disp == Invalid_Float) {
                mismatches.emplace_back(i, j);
                continue;
            }

            // 根据视差值找到右影像上对应的同名像素
            const auto col_right = static_cast<sint32>(j - disp + 0.5);

            if (col_right >= 0 && col_right < width) {
                // 右影像上同名像素的视差值
                const auto& disp_r = disp_right_[i * width + col_right];

                // 判断两个视差值是否一致（差值在阈值内）
                if (abs(disp - disp_r) > threshold) {
                    // 区分遮挡区和误匹配区
                    // 通过右影像视差算出在左影像的匹配像素，并获取视差disp_rl
                    // if(disp_rl > disp) 
                    //		pixel in occlusions
                    // else 
                    //		pixel in mismatches
                    const sint32 col_rl = static_cast<sint32>(col_right + disp_r + 0.5);
                    if (col_rl > 0 && col_rl < width) {
                        const auto& disp_l = disp_left_[i * width + col_rl];
                        if (disp_l > disp) {
                            occlusions.emplace_back(i, j);
                        }
                        else {
                            mismatches.emplace_back(i, j);
                        }
                    }
                    else {
                        mismatches.emplace_back(i, j);
                    }

                    // 让视差值无效
                    disp = Invalid_Float;
                }
            }
            else {
                // 通过视差值在右影像上找不到同名像素（超出影像范围）
                disp = Invalid_Float;
                mismatches.emplace_back(i, j);
            }
        }
    }
}

void SemiGlobalMatching::FillHolesInDispMap()
{
    const sint32 width = width_;
    const sint32 height = height_;

    std::vector<float32> disp_collects;

    // 定义8个方向
    float32 pi = 3.1415926;
    float32 angle1[8] = { pi, 3 * pi / 4, pi / 2, pi / 4, 0, 7 * pi / 4, 3 * pi / 2, 5 * pi / 4 };
    float32 angle2[8] = { pi, 5 * pi / 4, 3 * pi / 2, 7 * pi / 4, 0, pi / 4, pi / 2, 3 * pi / 4 };
    float32* angle = angle1;

    float32* disp_ptr = disp_left_;
    for (int k = 0; k < 3; k++) {
        // 第一次循环处理遮挡区，第二次循环处理误匹配区
        auto& trg_pixels = (k == 0) ? occlusions_ : mismatches_;

        std::vector<std::pair<int, int>> inv_pixels;
        if (k == 2) {
            //  第三次循环处理前两次没有处理干净的像素
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    if (disp_ptr[i * width + j] == Invalid_Float) {
                        inv_pixels.emplace_back(i, j);
                    }
                }
            }
            trg_pixels = inv_pixels;
        }

        // 遍历待处理像素
        for (auto& pix : trg_pixels) {
            int y = pix.first;
            int x = pix.second;

            if (y == height / 2) {
                angle = angle2;
            }

            // 收集8个方向上遇到的首个有效视差值
            disp_collects.clear();
            for (sint32 n = 0; n < 8; n++) {
                const float32 ang = angle[n];
                const float32 sina = sin(ang);
                const float32 cosa = cos(ang);
                for (sint32 n = 1; ; n++) {
                    const sint32 yy = y + n * sina;
                    const sint32 xx = x + n * cosa;
                    if (yy < 0 || yy >= height || xx < 0 || xx >= width) {
                        break;
                    }
                    auto& disp = *(disp_ptr + yy * width + xx);
                    if (disp != Invalid_Float) {
                        disp_collects.push_back(disp);
                        break;
                    }
                }
            }
            if (disp_collects.empty()) {
                continue;
            }

            std::sort(disp_collects.begin(), disp_collects.end());

            // 如果是遮挡区，则选择第二小的视差值
            // 如果是误匹配区，则选择中值
            if (k == 0) {
                if (disp_collects.size() > 1) {
                    disp_ptr[y * width + x] = disp_collects[1];
                }
                else {
                    disp_ptr[y * width + x] = disp_collects[0];
                }
            }
            else {
                disp_ptr[y * width + x] = disp_collects[disp_collects.size() / 2];
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
    //cv::Mat disp_norm;
    //cv::normalize(disp_mat, disp_norm, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Save disparity map
    std::string output_path = output_directory + "/disparity.png";
    if (!cv::imwrite(output_path, disp_mat)) {
        std::cout << "Failed to save disparity image to " << output_path << std::endl;
        delete[] disparity;
        return -1;
    }

    //cv::imshow("Disparity Map", disp_norm);
    cv::imshow("视差图", disp_mat);
    cv::waitKey(0);

    delete[] disparity;
    disparity = nullptr;

    return 0;
}

