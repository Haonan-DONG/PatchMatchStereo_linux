/* -*-c++-*- PatchMatchStereo - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
*			  https://github.com/ethan-li-coding
* Describe	: implement of cost computer
*/

#ifndef PATCH_MATCH_STEREO_COST_HPP_
#define PATCH_MATCH_STEREO_COST_HPP_
#include "pms_types.h"
#include <algorithm>

#define COST_PUNISH 120.0f // NOLINT(cppcoreguidelines-macro-usage)

#define USE_FAST_EXP
/* ����exp*/
inline double fast_exp(double x)
{
	x = 1.0 + x / 1024;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	x *= x;
	return x;
}

/**
 * \brief ���ۼ���������
 */
class CostComputer
{
public:
	/** \brief ���ۼ�����Ĭ�Ϲ��� */
	CostComputer() : img_left_(nullptr), img_right_(nullptr), width_(0), height_(0), patch_size_(0), min_disp_(0),
					 max_disp_(0) {}

	/**
	 * \brief ���ۼ�������ʼ��
	 * \param img_left		��Ӱ������ 
	 * \param img_right		��Ӱ������
	 * \param width			Ӱ���
	 * \param height		Ӱ���
	 * \param patch_size	�ֲ�Patch��С
	 * \param min_disp		��С�Ӳ�
	 * \param max_disp		����Ӳ�
	 */
	CostComputer(const uint8 *img_left, const uint8 *img_right, const sint32 &width, const sint32 &height, const sint32 &patch_size, const sint32 &min_disp, const sint32 &max_disp)
	{
		img_left_ = img_left;
		img_right_ = img_right;
		width_ = width;
		height_ = height;
		patch_size_ = patch_size;
		min_disp_ = min_disp;
		max_disp_ = max_disp;
	}

	/** \brief ���ۼ��������� */
	virtual ~CostComputer() = default;

public:
	/**
	 * \brief ������Ӱ��p���Ӳ�Ϊdʱ�Ĵ���ֵ
	 * \param i		p��������
	 * \param j		p�������
	 * \param d		�Ӳ�ֵ
	 * \return ����ֵ
	 */
	virtual float32 Compute(const sint32 &i, const sint32 &j, const float32 &d) = 0;

public:
	/** \brief ��Ӱ������ */
	const uint8 *img_left_;
	/** \brief ��Ӱ������ */
	const uint8 *img_right_;

	/** \brief Ӱ��� */
	sint32 width_;
	/** \brief Ӱ��� */
	sint32 height_;
	/** \brief �ֲ�����Patch��С */
	sint32 patch_size_;

	/** \brief ��С����Ӳ� */
	sint32 min_disp_;
	sint32 max_disp_;
};

/**
 * \brief ���ۼ�������PatchMatchSteroԭ�Ĵ��ۼ�����
 */
class CostComputerPMS : public CostComputer
{
public:
	/** \brief PMS���ۼ�����Ĭ�Ϲ��� */
	CostComputerPMS() : grad_left_(nullptr), grad_right_(nullptr), gamma_(0), alpha_(0), tau_col_(0), tau_grad_(0){};

	/**
	 * \brief PMS���ۼ��������ι���
	 * \param img_left		��Ӱ������
	 * \param img_right		��Ӱ������
	 * \param grad_left		���ݶ�����
	 * \param grad_right	���ݶ�����
	 * \param width			Ӱ���
	 * \param height		Ӱ���
	 * \param patch_size	�ֲ�Patch��С
	 * \param min_disp		��С�Ӳ�
	 * \param max_disp		����Ӳ�
	 * \param gamma			����gammaֵ
	 * \param alpha			����alphaֵ
	 * \param t_col			����tau_colֵ
	 * \param t_grad		����tau_gradֵ
	 */
	CostComputerPMS(const uint8 *img_left, const uint8 *img_right, const PGradient *grad_left, const PGradient *grad_right, const sint32 &width, const sint32 &height, const sint32 &patch_size,
					const sint32 &min_disp, const sint32 &max_disp,
					const float32 &gamma, const float32 &alpha, const float32 &t_col, const float32 t_grad) : CostComputer(img_left, img_right, width, height, patch_size, min_disp, max_disp)
	{
		grad_left_ = grad_left;
		grad_right_ = grad_right;
		gamma_ = gamma;
		alpha_ = alpha;
		tau_col_ = t_col;
		tau_grad_ = t_grad;
	}

	/**
	 * \brief ������Ӱ��p���Ӳ�Ϊdʱ�Ĵ���ֵ
	 * \param x		p��x����
	 * \param y		p��y����
	 * \param d		�Ӳ�ֵ
	 * \return ����ֵ
	 */
	inline float32 Compute(const sint32 &x, const sint32 &y, const float32 &d) override
	{
		const float32 xr = x - d;
		if (xr < 0.0f || xr >= static_cast<float32>(width_))
		{
			return (1 - alpha_) * tau_col_ + alpha_ * tau_grad_;
		}
		// ��ɫ�ռ����
		const auto col_p = GetColor(img_left_, x, y);
		const auto col_q = GetColor(img_right_, xr, y);
		const auto dc = std::min(abs(col_p.b - col_q.x) + abs(col_p.g - col_q.y) + abs(col_p.r - col_q.z), tau_col_);

		// �ݶȿռ����
		const auto grad_p = GetGradient(grad_left_, x, y);
		const auto grad_q = GetGradient(grad_right_, xr, y);
		const auto dg = std::min(abs(grad_p.x - grad_q.x) + abs(grad_p.y - grad_q.y), tau_grad_);

		// ����ֵ
		return (1 - alpha_) * dc + alpha_ * dg;
	}

	/**
	 * \brief ������Ӱ��p���Ӳ�Ϊdʱ�Ĵ���ֵ
	 * \param col_p		p����ɫֵ
	 * \param grad_p	p���ݶ�ֵ
	 * \param x			p��x����
	 * \param y			p��y����
	 * \param d			�Ӳ�ֵ
	 * \return ����ֵ
	 */
	inline float32 Compute(const PColor &col_p, const PGradient &grad_p, const sint32 &x, const sint32 &y, const float32 &d) const
	{
		const float32 xr = x - d;
		if (xr < 0.0f || xr >= static_cast<float32>(width_))
		{
			return (1 - alpha_) * tau_col_ + alpha_ * tau_grad_;
		}
		// ��ɫ�ռ����
		const auto col_q = GetColor(img_right_, xr, y);
		const auto dc = std::min(abs(col_p.b - col_q.x) + abs(col_p.g - col_q.y) + abs(col_p.r - col_q.z), tau_col_);

		// �ݶȿռ����
		const auto grad_q = GetGradient(grad_right_, xr, y);
		const auto dg = std::min(abs(grad_p.x - grad_q.x) + abs(grad_p.y - grad_q.y), tau_grad_);

		// ����ֵ
		return (1 - alpha_) * dc + alpha_ * dg;
	}

	/**
	 * \brief ������Ӱ��p���Ӳ�ƽ��Ϊpʱ�ľۺϴ���ֵ
	 * \param x		p��x����
	 * \param y 	p��y����
	 * \param p		ƽ�����
	 * \return �ۺϴ���ֵ
	 */
	inline float32 ComputeA(const sint32 &x, const sint32 &y, const DisparityPlane &p) const
	{
		const auto pat = patch_size_ / 2;
		const auto &col_p = GetColor(img_left_, x, y);
		float32 cost = 0.0f;
		for (sint32 r = -pat; r <= pat; r++)
		{
			const sint32 yr = y + r;
			for (sint32 c = -pat; c <= pat; c++)
			{
				const sint32 xc = x + c;
				if (yr < 0 || yr > height_ - 1 || xc < 0 || xc > width_ - 1)
				{
					continue;
				}
				// �����Ӳ�ֵ
				const float32 d = p.to_disparity(xc, yr);
				if (d < min_disp_ || d > max_disp_)
				{
					cost += COST_PUNISH;
					continue;
				}

				// ����Ȩֵ
				const auto &col_q = GetColor(img_left_, xc, yr);
				const auto dc = abs(col_p.r - col_q.r) + abs(col_p.g - col_q.g) + abs(col_p.b - col_q.b);
#ifdef USE_FAST_EXP
				const auto w = fast_exp(double(-dc / gamma_));
#else
				const auto w = exp(-dc / gamma_);
#endif

				// �ۺϴ���
				const auto grad_q = GetGradient(grad_left_, xc, yr);
				cost += w * Compute(col_q, grad_q, xc, yr, d);
			}
		}
		return cost;
	}

	/**
	* \brief ��ȡ���ص����ɫֵ
	* \param img_data	��ɫ����,3ͨ��
	* \param x			����x����
	* \param y			����y����
	* \return ����(x,y)����ɫֵ
	*/
	inline PColor GetColor(const uint8 *img_data, const sint32 &x, const sint32 &y) const
	{
		auto *pixel = img_data + y * width_ * 3 + 3 * x;
		return {pixel[0], pixel[1], pixel[2]};
	}

	/**
	* \brief ��ȡ���ص����ɫֵ
	* \param img_data	��ɫ����
	* \param x			����x���꣬ʵ���������ڲ�õ���ɫֵ
	* \param y			����y����
	* \return ����(x,y)����ɫֵ
	*/
	inline PVector3f GetColor(const uint8 *img_data, const float32 &x, const sint32 &y) const
	{
		float32 col[3];
		const auto x1 = static_cast<sint32>(x);
		const sint32 x2 = x1 + 1;
		const float32 ofs = x - x1;

		for (sint32 n = 0; n < 3; n++)
		{
			const auto &g1 = img_data[y * width_ * 3 + 3 * x1 + n];
			const auto &g2 = (x2 < width_) ? img_data[y * width_ * 3 + 3 * x2 + n] : g1;
			col[n] = (1 - ofs) * g1 + ofs * g2;
		}

		return {col[0], col[1], col[2]};
	}

	/**
	* \brief ��ȡ���ص���ݶ�ֵ
	* \param grad_data	�ݶ�����
	* \param x			����x����
	* \param y			����y����
	* \return ����(x,y)���ݶ�ֵ
	*/
	inline PGradient GetGradient(const PGradient *grad_data, const sint32 &x, const sint32 &y) const
	{
		return grad_data[y * width_ + x];
	}

	/**
	* \brief ��ȡ���ص���ݶ�ֵ
	* \param grad_data	�ݶ�����
	* \param x			����x���꣬ʵ���������ڲ�õ��ݶ�ֵ
	* \param y			����y����
	* \return ����(x,y)���ݶ�ֵ
	*/
	inline PVector2f GetGradient(const PGradient *grad_data, const float32 &x, const sint32 &y) const
	{
		const auto x1 = static_cast<sint32>(x);
		const sint32 x2 = x1 + 1;
		const float32 ofs = x - x1;

		const auto &g1 = grad_data[y * width_ + x1];
		const auto &g2 = (x2 < width_) ? grad_data[y * width_ + x2] : g1;

		return {(1 - ofs) * g1.x + ofs * g2.x, (1 - ofs) * g1.y + ofs * g2.y};
	}

private:
	/** \brief ��Ӱ���ݶ����� */
	const PGradient *grad_left_;
	/** \brief ��Ӱ���ݶ����� */
	const PGradient *grad_right_;

	/** \brief ����gamma */
	float gamma_;
	/** \brief ����alpha */
	float32 alpha_;
	/** \brief ����tau_col */
	float32 tau_col_;
	/** \brief ����tau_grad */
	float32 tau_grad_;
};

// �������ڴ�ͨ��������ķ�ʽʵ������ʵ�ֵĴ��ۼ�����������

#endif
