#ifndef __UTILS_VISUAL_SERVOING_H__
#define __UTILS_VISUAL_SERVOING_H__

#include <Eigen/Core>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpFeaturePoint.h>
#include <list>
#include <opensot_visual_servoing/VisualFeatures.h>
#include <opensot_visual_servoing/VisualFeature.h>

namespace opensot_visual_servoing {
namespace utils {

template<typename Derived>
/**
 * @brief visp2eigen convert a visp matrix in eigen matrix
 * @param src vpMatrix
 * @param dst eigen matrix
 */
void visp2eigen(const vpMatrix &src, Eigen::MatrixBase<Derived> &dst)
{
    dst = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(src.data, src.getRows(), src.getCols());
}

template<typename Derived>
/**
 * @brief visp2eigen convert a visp vector in eigen vector
 * @param src vpColVector
 * @param dst eigen vector
 */
void visp2eigen(const vpColVector& src, Eigen::MatrixBase<Derived> &dst)
{
    dst = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1> >(src.data, src.getRows());
}

template<typename Derived>
void eigen2visp(const Eigen::MatrixXd &src, vpHomogeneousMatrix &dst)
{
    if (src.rows() != 4 || src.cols() != 4) {
        throw vpException(vpException::dimensionError, "Input Eigen Matrix must be of size (4,4)!");
}

Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> >(&dst.data[0], src.rows(), src.cols()) = src;
}

template<typename Derived>
void eigen2visp(const Eigen::VectorXd &src, vpColVector &dst)
{
    dst.resize(static_cast<unsigned int>(src.rows()));
    #if (VP_VERSION_INT(EIGEN_WORLD_VERSION, EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION) < 0x030300)
    for (Eigen::DenseIndex i = 0; i < src.rows(); i++) {
    #else
    for (Eigen::Index i = 0; i < src.rows(); i++) {
    #endif
        dst[static_cast<unsigned int>(i)] = src(i);
    }
}

template<typename FeatureType>
/**
 * @brief toGenericFeature
 * @param feature_list
 * @return
 */
std::list<vpBasicFeature*> toGenericFeature(const std::list<FeatureType*>& feature_list)
{
    std::list<vpBasicFeature *> generic_features(std::begin(feature_list), std::end(feature_list));
    return generic_features;
}

/**
 * @brief toVisualFeatureMsg transform a list of visual feature in a visual feature msg
 * @param feature_list list of features
 * @param features_type type of feature (point, line, etc...)
 * @return pensot_visual_servoing::VisualFeatures (normalized coordinates)
 */
opensot_visual_servoing::VisualFeatures toVisualFeatureMsg(
    const std::list<vpBasicFeature * >& feature_list,
    const std::string& features_type = "vpFeaturePoint");

/**
 * @brief getFeaturesFromMsg transform a feature msg in a list of features
 * @param msg visaul feature msg (in normalized coordinates)
 * @return list of features (in normalized coordinates)
 */
std::list<vpBasicFeature*> getFeaturesFromMsg(opensot_visual_servoing::VisualFeaturesConstPtr msg);

} // namespace utils
} // namespace opensot_visual_servoing
#endif
