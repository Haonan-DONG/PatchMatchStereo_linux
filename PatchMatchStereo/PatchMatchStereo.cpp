/* -*-c++-*- PatchMatchStereo - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
*			  https://github.com/ethan-li-coding
* Describe	: implement of patch match stereo class
*/

#include "stdafx.h"
#include <iostream>
#include "PatchMatchStereo.h"
#include <ctime>
#include <random>
#include <cmath>
#include "pms_propagation.h"
#include "pms_util.h"

PatchMatchStereo::PatchMatchStereo() : width_(0), height_(0), img_left_(nullptr), img_right_(nullptr),
									   gray_left_(nullptr), gray_right_(nullptr),
									   grad_left_(nullptr), grad_right_(nullptr),
									   cost_left_(nullptr), cost_right_(nullptr),
									   disp_left_(nullptr), disp_right_(nullptr),
									   plane_left_(nullptr), plane_right_(nullptr),
									   is_initialized_(false) {}

PatchMatchStereo::~PatchMatchStereo()
{
	Release();
}

bool PatchMatchStereo::Initialize(const sint32 &width, const sint32 &height, const PMSOption &option)
{
	// ������ ��ֵ

	// Ӱ��ߴ�
	width_ = width;
	height_ = height;
	// PMS����
	option_ = option;

	if (width <= 0 || height <= 0)
	{
		return false;
	}

	//������ �����ڴ�ռ�
	const sint32 img_size = width * height;
	const sint32 disp_range = option.max_disparity - option.min_disparity;
	// �Ҷ�����
	gray_left_ = new uint8[img_size];
	gray_right_ = new uint8[img_size];
	// �ݶ�����
	grad_left_ = new PGradient[img_size]();
	grad_right_ = new PGradient[img_size]();
	// ��������
	cost_left_ = new float32[img_size];
	cost_right_ = new float32[img_size];
	// �Ӳ�ͼ
	disp_left_ = new float32[img_size];
	disp_right_ = new float32[img_size];
	// ƽ�漯
	plane_left_ = new DisparityPlane[img_size];
	plane_right_ = new DisparityPlane[img_size];

	is_initialized_ = grad_left_ && grad_right_ && disp_left_ && disp_right_ && plane_left_ && plane_right_;

	return is_initialized_;
}

void PatchMatchStereo::Release()
{
	SAFE_DELETE(grad_left_);
	SAFE_DELETE(grad_right_);
	SAFE_DELETE(cost_left_);
	SAFE_DELETE(cost_right_);
	SAFE_DELETE(disp_left_);
	SAFE_DELETE(disp_right_);
	SAFE_DELETE(plane_left_);
	SAFE_DELETE(plane_right_);
}

bool PatchMatchStereo::Match(const uint8 *img_left, const uint8 *img_right, float32 *disp_left)
{
	if (!is_initialized_)
	{
		return false;
	}
	if (img_left == nullptr || img_right == nullptr)
	{
		return false;
	}

	img_left_ = img_left;
	img_right_ = img_right;

	std::cout << "Start to randomly initialize the patch match." << std::endl;
	RandomInitialization();

	std::cout << "Start to compute gray" << std::endl;
	ComputeGray();
	
	std::cout << "Start to compute the gradient" << std::endl;
	ComputeGradient();
	
	std::cout << "Start to propagation." << std::endl;
	Propagation();
	
	std::cout << "Start to plane to disparity." << std::endl;
	PlaneToDisparity();
	
	if (option_.is_check_lr)
	{
		// һ���Լ��
		LRCheck();
	}

	// �Ӳ����
	if (option_.is_fill_holes)
	{
		FillHolesInDispMap();
	}

	// ����Ӳ�ͼ
	if (disp_left && disp_left_)
	{
		memcpy(disp_left, disp_left_, height_ * width_ * sizeof(float32));
	}
	return true;
}

bool PatchMatchStereo::Reset(const uint32 &width, const uint32 &height, const PMSOption &option)
{
	// �ͷ��ڴ�
	Release();

	// ���ó�ʼ�����
	is_initialized_ = false;

	return Initialize(width, height, option);
}

float *PatchMatchStereo::GetDisparityMap(const sint32 &view) const
{
	switch (view)
	{
	case 0:
		return disp_left_;
	case 1:
		return disp_right_;
	default:
		return nullptr;
	}
}

PGradient *PatchMatchStereo::GetGradientMap(const sint32 &view) const
{
	switch (view)
	{
	case 0:
		return grad_left_;
	case 1:
		return grad_right_;
	default:
		return nullptr;
	}
}

void PatchMatchStereo::RandomInitialization() const
{
	const sint32 width = width_;
	const sint32 height = height_;
	if (width <= 0 || height <= 0 ||
		disp_left_ == nullptr || disp_right_ == nullptr ||
		plane_left_ == nullptr || plane_right_ == nullptr)
	{
		return;
	}
	const auto &option = option_;
	const sint32 min_disparity = option.min_disparity;
	const sint32 max_disparity = option.max_disparity;

	// �����������
	// Generate random number.
	std::random_device rd;
	//std::mt19937 gen(rd());
	std::mt19937 gen;
	gen.seed(rd());
	// we dismiss the const for the random distribution.
	std::uniform_real_distribution<float32> rand_d(static_cast<float32>(min_disparity), static_cast<float32>(max_disparity));
	std::uniform_real_distribution<float32> rand_n(-1.0f, 1.0f);

	for (int k = 0; k < 2; k++)
	{
		auto *disp_ptr = k == 0 ? disp_left_ : disp_right_;
		auto *plane_ptr = k == 0 ? plane_left_ : plane_right_;
		sint32 sign = (k == 0) ? 1 : -1;
		for (sint32 y = 0; y < height; y++)
		{
			for (sint32 x = 0; x < width; x++)
			{
				const sint32 p = y * width + x;
				// ����Ӳ�ֵ
				float32 disp = sign * rand_d(gen);
				if (option.is_integer_disp)
				{
					disp = static_cast<float32>(round(disp));
				}
				disp_ptr[p] = disp;

				// ���������
				PVector3f norm;
				if (!option.is_fource_fpw)
				{
					norm.x = rand_n(gen);
					norm.y = rand_n(gen);
					float32 z = rand_n(gen);
					while (z == 0.0f)
					{
						z = rand_n(gen);
					}
					norm.z = z;
					norm.normalize();
				}
				else
				{
					norm.x = 0.0f;
					norm.y = 0.0f;
					norm.z = 1.0f;
				}

				// �����Ӳ�ƽ��
				plane_ptr[p] = DisparityPlane(x, y, norm, disp);
			}
		}
	}
}

void PatchMatchStereo::ComputeGray() const
{
	const sint32 width = width_;
	const sint32 height = height_;
	if (width <= 0 || height <= 0 ||
		img_left_ == nullptr || img_right_ == nullptr ||
		gray_left_ == nullptr || gray_right_ == nullptr)
	{
		return;
	}

	// ��ɫת�Ҷ�
	for (sint32 n = 0; n < 2; n++)
	{
		auto *color = (n == 0) ? img_left_ : img_right_;
		auto *gray = (n == 0) ? gray_left_ : gray_right_;
		for (sint32 i = 0; i < height; i++)
		{
			for (sint32 j = 0; j < width; j++)
			{
				const auto b = color[i * width * 3 + 3 * j];
				const auto g = color[i * width * 3 + 3 * j + 1];
				const auto r = color[i * width * 3 + 3 * j + 2];
				gray[i * width + j] = uint8(r * 0.299 + g * 0.587 + b * 0.114);
			}
		}
	}
}

void PatchMatchStereo::ComputeGradient() const
{
	const sint32 width = width_;
	const sint32 height = height_;
	if (width <= 0 || height <= 0 ||
		grad_left_ == nullptr || grad_right_ == nullptr ||
		gray_left_ == nullptr || gray_right_ == nullptr)
	{
		return;
	}

	// Sobel�ݶ�����
	for (sint32 n = 0; n < 2; n++)
	{
		auto *gray = (n == 0) ? gray_left_ : gray_right_;
		auto *grad = (n == 0) ? grad_left_ : grad_right_;
		for (int y = 1; y < height - 1; y++)
		{
			for (int x = 1; x < width - 1; x++)
			{
				const auto grad_x = (-gray[(y - 1) * width + x - 1] + gray[(y - 1) * width + x + 1]) +
									(-2 * gray[y * width + x - 1] + 2 * gray[y * width + x + 1]) +
									(-gray[(y + 1) * width + x - 1] + gray[(y + 1) * width + x + 1]);
				const auto grad_y = (-gray[(y - 1) * width + x - 1] - 2 * gray[(y - 1) * width + x] - gray[(y - 1) * width + x + 1]) +
									(gray[(y + 1) * width + x - 1] + 2 * gray[(y + 1) * width + x] + gray[(y + 1) * width + x + 1]);

				// �������8��Ϊ�����ݶȵ����ֵ������255�������������ʱ�ݶȲ����ɫ��λ��ͬһ���߶�
				grad[y * width + x].x = grad_x / 8;
				grad[y * width + x].y = grad_y / 8;
			}
		}
	}
}

void PatchMatchStereo::Propagation() const
{
	const sint32 width = width_;
	const sint32 height = height_;
	if (width <= 0 || height <= 0 ||
		img_left_ == nullptr || img_right_ == nullptr ||
		grad_left_ == nullptr || grad_right_ == nullptr ||
		disp_left_ == nullptr || disp_right_ == nullptr ||
		plane_left_ == nullptr || plane_right_ == nullptr)
	{
		return;
	}

	// ������ͼƥ�����
	const auto opion_left = option_;
	auto option_right = option_;
	option_right.min_disparity = -opion_left.max_disparity;
	option_right.max_disparity = -opion_left.min_disparity;

	// ������ͼ����ʵ��
	PMSPropagation propa_left(width, height, img_left_, img_right_, grad_left_, grad_right_, plane_left_, plane_right_, opion_left, cost_left_, cost_right_, disp_left_);
	PMSPropagation propa_right(width, height, img_right_, img_left_, grad_right_, grad_left_, plane_right_, plane_left_, option_right, cost_right_, cost_left_, disp_right_);

	// ��������
	for (int k = 0; k < option_.num_iters; k++)
	{
		printf("Iteration %d in total %d.\n", k, option_.num_iters);
		propa_left.DoPropagation();
		propa_right.DoPropagation();
	}
}

void PatchMatchStereo::LRCheck()
{
	const sint32 width = width_;
	const sint32 height = height_;

	const float32 &threshold = option_.lrcheck_thres;

	// k==0 : ����ͼһ���Լ��
	// k==1 : ����ͼһ���Լ��
	for (int k = 0; k < 2; k++)
	{
		auto *disp_left = (k == 0) ? disp_left_ : disp_right_;
		auto *disp_right = (k == 0) ? disp_right_ : disp_left_;
		auto &mismatches = (k == 0) ? mismatches_left_ : mismatches_right_;
		mismatches.clear();

		// ---����һ���Լ��
		for (sint32 y = 0; y < height; y++)
		{
			for (sint32 x = 0; x < width; x++)
			{

				// ��Ӱ���Ӳ�ֵ
				auto &disp = disp_left[y * width + x];

				if (disp == Invalid_Float)
				{
					mismatches.emplace_back(x, y);
					continue;
				}

				// �����Ӳ�ֵ�ҵ���Ӱ���϶�Ӧ��ͬ������
				const auto col_right = lround(x - disp);

				if (col_right >= 0 && col_right < width)
				{
					// ��Ӱ����ͬ�����ص��Ӳ�ֵ
					auto &disp_r = disp_right[y * width + col_right];

					// �ж������Ӳ�ֵ�Ƿ�һ�£���ֵ����ֵ��Ϊһ�£�
					// �ڱ������������ͼ���Ӳ�ֵ�����෴
					if (abs(disp + disp_r) > threshold)
					{
						// ���Ӳ�ֵ��Ч
						disp = Invalid_Float;
						mismatches.emplace_back(x, y);
					}
				}
				else
				{
					// ͨ���Ӳ�ֵ����Ӱ�����Ҳ���ͬ�����أ�����Ӱ��Χ��
					disp = Invalid_Float;
					mismatches.emplace_back(x, y);
				}
			}
		}
	}
}

void PatchMatchStereo::FillHolesInDispMap()
{
	const sint32 width = width_;
	const sint32 height = height_;
	if (width <= 0 || height <= 0 ||
		disp_left_ == nullptr || disp_right_ == nullptr ||
		plane_left_ == nullptr || plane_right_ == nullptr)
	{
		return;
	}

	const auto &option = option_;

	// k==0 : ����ͼ�Ӳ����
	// k==1 : ����ͼ�Ӳ����
	for (int k = 0; k < 2; k++)
	{
		auto &mismatches = (k == 0) ? mismatches_left_ : mismatches_right_;
		if (mismatches.empty())
		{
			continue;
		}
		const auto *img_ptr = (k == 0) ? img_left_ : img_right_;
		const auto *plane_ptr = (k == 0) ? plane_left_ : plane_right_;
		auto *disp_ptr = (k == 0) ? disp_left_ : disp_right_;
		vector<float32> fill_disps(mismatches.size()); // �洢ÿ����������ص��Ӳ�
		for (auto n = 0u; n < mismatches.size(); n++)
		{
			auto &pix = mismatches[n];
			const sint32 x = pix.first;
			const sint32 y = pix.second;
			vector<DisparityPlane> planes;

			// �������Ҹ���Ѱ��һ����Ч���أ���¼ƽ��
			sint32 xs = x + 1;
			while (xs < width)
			{
				if (disp_ptr[y * width + xs] != Invalid_Float)
				{
					planes.push_back(plane_ptr[y * width + xs]);
					break;
				}
				xs++;
			}
			xs = x - 1;
			while (xs >= 0)
			{
				if (disp_ptr[y * width + xs] != Invalid_Float)
				{
					planes.push_back(plane_ptr[y * width + xs]);
					break;
				}
				xs--;
			}

			if (planes.empty())
			{
				continue;
			}
			else if (planes.size() == 1u)
			{
				fill_disps[n] = planes[0].to_disparity(x, y);
			}
			else
			{
				// ѡ���С���Ӳ�
				const auto d1 = planes[0].to_disparity(x, y);
				const auto d2 = planes[1].to_disparity(x, y);
				fill_disps[n] = abs(d1) < abs(d2) ? d1 : d2;
			}
		}
		for (auto n = 0u; n < mismatches.size(); n++)
		{
			auto &pix = mismatches[n];
			const sint32 x = pix.first;
			const sint32 y = pix.second;
			disp_ptr[y * width + x] = fill_disps[n];
		}

		// ��Ȩ��ֵ�˲�
		pms_util::WeightedMedianFilter(img_ptr, width, height, option.patch_size, option.gamma, mismatches, disp_ptr);
	}
}

void PatchMatchStereo::PlaneToDisparity() const
{
	const sint32 width = width_;
	const sint32 height = height_;
	if (width <= 0 || height <= 0 ||
		disp_left_ == nullptr || disp_right_ == nullptr ||
		plane_left_ == nullptr || plane_right_ == nullptr)
	{
		return;
	}
	for (int k = 0; k < 2; k++)
	{
		auto *plane_ptr = (k == 0) ? plane_left_ : plane_right_;
		auto *disp_ptr = (k == 0) ? disp_left_ : disp_right_;
		for (sint32 y = 0; y < height; y++)
		{
			for (sint32 x = 0; x < width; x++)
			{
				const sint32 p = y * width + x;
				const auto &plane = plane_ptr[p];
				disp_ptr[p] = plane.to_disparity(x, y);
			}
		}
	}
}
