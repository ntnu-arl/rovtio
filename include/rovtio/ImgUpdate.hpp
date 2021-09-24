/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ROVIO_IMGUPDATE_HPP_
#define ROVIO_IMGUPDATE_HPP_

#include "lightweight_filtering/common.hpp"
#include "lightweight_filtering/Update.hpp"
#include "lightweight_filtering/State.hpp"

#include "rovtio/CoordinateTransform/FeatureOutput.hpp"
#include "rovtio/FilterStates.hpp"
#include "rovtio/Camera.hpp"
#include "rovtio/CoordinateTransform/PixelOutput.hpp"
#include "rovtio/ZeroVelocityUpdate.hpp"
#include "rovtio/MultilevelPatchAlignment.hpp"
//#include <functional> // Custom to sort
#include <algorithm> // Custom to sort
#include <chrono>
#include <array>

namespace rovtio {

/** \brief Class, defining the innovation.
 *
 *  @tparam STATE - Filter State
 */
  template<typename STATE>
  class ImgInnovation : public LWF::State<LWF::VectorElement<2>> {
  public:
    typedef LWF::State<LWF::VectorElement<2>> Base;
    using Base::E_;
    static constexpr unsigned int _pix = 0;

    ImgInnovation() {
      static_assert(_pix + 1 == E_, "Error with indices");
      this->template getName<_pix>() = "pix";
    };

    virtual ~ImgInnovation() {};
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding the image pyramids of the different cameras.
 *
 *  @tparam STATE - Filter State
 */
  template<typename STATE>
  class ImgUpdateMeasAuxiliary : public LWF::AuxiliaryBase<ImgUpdateMeasAuxiliary<STATE>> {
  public:
    ImgUpdateMeasAuxiliary() {
      reset(0.0);
    };

    virtual ~ImgUpdateMeasAuxiliary() {};

    void reset(const double t) {
      imgTime_ = t;
    }

    ImagePyramid<STATE::nLevels_> pyr_[STATE::nCam_];
    double imgTime_;
    int activeModality_; // The camera ID of the camera that took the image which is being updated.
    bool camActive_[STATE::nCam_]; // Custom to track which cameras are receiving new images
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**  \brief Update measurement class (all data in auxillary)
 *
 *  @tparam STATE - Filter State
 */
  template<typename STATE>
  class ImgUpdateMeas : public LWF::State<ImgUpdateMeasAuxiliary<STATE>> {
  public:
    typedef LWF::State<ImgUpdateMeasAuxiliary<STATE>> Base;
    using Base::E_;
    static constexpr unsigned int _aux = 0;

    ImgUpdateMeas() {
      static_assert(_aux + 1 == E_, "Error with indices");
    };

    virtual ~ImgUpdateMeas() {};

    //@{
    /** \brief Get the auxiliary state of the ImgUpdateMeas.
     *
     *  \see ImgUpdateMeasAuxiliary
     *  @return the the auxiliary state of the ImgUpdateMeas.
     */
    inline ImgUpdateMeasAuxiliary<STATE> &aux() {
      return this->template get<_aux>();
    }

    inline const ImgUpdateMeasAuxiliary<STATE> &aux() const {
      return this->template get<_aux>();
    }
    //@}
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**  \brief Class holding the update noise.
 *
 *  @tparam STATE - Filter State
 */
  template<typename STATE>
  class ImgUpdateNoise : public LWF::State<LWF::VectorElement<2>> {
  public:
    typedef LWF::State<LWF::VectorElement<2>> Base;
    using Base::E_;
    static constexpr unsigned int _pix = 0;

    ImgUpdateNoise() {
      static_assert(_pix + 1 == E_, "Error with indices");
      this->template getName<_pix>() = "pix";
    };

    virtual ~ImgUpdateNoise() {};
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Outlier Detection.
 *
 *  @tparam STATE - Filter State
 */
  template<typename STATE>
  class ImgOutlierDetection
          : public LWF::OutlierDetection<LWF::ODEntry<ImgInnovation<STATE>::template getId<ImgInnovation<STATE>::_pix>(), 2>> {
  public:
    /** \brief Destructor
     */
    virtual ~ImgOutlierDetection() {};
  };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** \brief Class, holding image update routines for the filter.
 */
  template<typename FILTERSTATE>
  class ImgUpdate
          : public LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>, FILTERSTATE, ImgUpdateMeas<typename FILTERSTATE::mtState>, ImgUpdateNoise<typename FILTERSTATE::mtState>,
                  ImgOutlierDetection<typename FILTERSTATE::mtState>, false> {
  public:
    typedef LWF::Update<ImgInnovation<typename FILTERSTATE::mtState>, FILTERSTATE, ImgUpdateMeas<typename FILTERSTATE::mtState>, ImgUpdateNoise<typename FILTERSTATE::mtState>,
            ImgOutlierDetection<typename FILTERSTATE::mtState>, false> Base;
    using Base::meas_;
    using Base::doubleRegister_;
    using Base::intRegister_;
    using Base::boolRegister_;
    using Base::updnoiP_;
    using Base::useImprovedJacobian_;
    using Base::hasConverged_;
    using Base::successfulUpdate_;
    using Base::cancelIteration_;
    using Base::candidateCounter_;
    typedef typename Base::mtState mtState;
    typedef typename Base::mtFilterState mtFilterState;
    typedef typename Base::mtInnovation mtInnovation;
    typedef typename Base::mtMeas mtMeas;
    typedef typename Base::mtNoise mtNoise;
    typedef typename Base::mtOutlierDetection mtOutlierDetection;

    // Multicamera pointer
    MultiCamera<mtState::nCam_> *mpMultiCamera_;

    // Parameter
    M3D initCovFeature_;
    double initDepth_;
    int startLevel_;
    int endLevel_;
    double startDetectionTh_;
    int nDetectionBuckets_;
    std::array<int, mtState::nCam_> fastDetectionThreshold_;
    double scoreDetectionExponent_;
    double penaltyDistance_;
    double zeroDistancePenalty_;
    double trackingUpperBound_, trackingLowerBound_;
    double minTrackedAndFreeFeatures_;
    double minRelativeSTScore_;
    std::array<double, mtState::nCam_> minAbsoluteSTScore_;
    double minTimeBetweenPatchUpdate_;
    bool doVisualMotionDetection_; /**<Do visual motion detection*/
    double rateOfMovingFeaturesTh_; /**<What percentage of feature must be moving for image motion detection*/
    double pixelCoordinateMotionTh_; /**<Threshold for detecting feature motion*/
    int minFeatureCountForNoMotionDetection_; /**<Minimum amount of feature for detecting NO image motion*/
    double removalFactor_; /**<Factor for enforcing feature removal if not enough free*/
    std::array<double, mtState::nCam_> patchRejectionTh_;
    bool useDirectMethod_;  /**<If true, the innovation term is based directly on pixel intensity errors.
                              If false, the reprojection error is used for the innovation term.*/
    bool doFrameVisualisation_;
    bool visualizePatches_;
    bool verbose_;
    bool removeNegativeFeatureAfterUpdate_;
    bool isZeroVelocityUpdateEnabled_; /**<Should zero velocity updates be performed*/
    double minTimeForZeroVelocityUpdate_;  /**<Time until zero velocity update get performed if there is no motion*/
    ZeroVelocityUpdate<FILTERSTATE> zeroVelocityUpdate_; /**<Zero velocity update, directly integrated into the img update*/
    double maxUncertaintyToDepthRatioForDepthInitialization_;
    double updateNoisePix_[mtState::nCam_]; /**<Update noise of update pixel, used for indirect case (reprojection error)*/
    double updateNoiseInt_[mtState::nCam_]; /**<Update noise of intensity error, used for direct approach*/
    double noiseGainForOffCamera_; /**<Factor added on update noise if not main camera*/
    double alignConvergencePixelRange_;
    double alignCoverageRatio_;
    int alignMaxUniSample_;
    bool useCrossCameraMeasurements_; /**<Should features be matched across cameras.*/
    bool doStereoInitialization_; /**<Should a stereo match be used for feature initialization.*/
    double alignmentHuberNormThreshold_; /**<Intensity error threshold for Huber norm.*/
    double alignmentGaussianWeightingSigma_; /**<Width of Gaussian which is used for pixel error weighting.*/
    double alignmentGradientExponent_; /**<Exponent used for gradient based weighting of residuals.*/
    double discriminativeSamplingDistance_; /**<Sampling distance for checking discriminativity of patch (if <= 0.0 no check is performed).*/
    double discriminativeSamplingGain_; /**<Gain for threshold above which the samples must lie (if <= 1.0 the patchRejectionTh is used).*/
    double maxAllowedFeatureDistance_;       /** Max allowed estimated feature distance after performing update, remove features with very far away depth and clustered features during backward motion.*/
    std::array<double, mtState::nCam_> additionalScoreFromOtherModality_; /** The scalar to be added to the score if it is close to a feature. */
    double maxDistanceForApproxamentlyLocalizingFeatures_; /** The maximum euclidian distance in pixel space which is close enough for "approximate localization". */
    std::array<int, mtState::nCam_> numFoundFeatures_; /** The number of features foundin each modality. */
    std::array<double, mtState::nCam_> featureCountHistory_; /** A score per camera outlining how nice this modality have been for feature tracking earlier. */
    std::array<double, mtState::nCam_> previousUpdateOfFeatureCountHistory_; /** A timestamp of the previous call in order to know how much we should update it with. */
    int framesSinceLastDetectionAtempt_[mtState::nCam_]; /** Number of frames since that modality is considered.*/
    double minSTScoreForModalitySelection_[mtState::nCam_]; /** Threshold for which features which are counted in the comparison for feature selection. */
    std::array<std::vector<FoundFeature>, mtState::nCam_> goodFeatures; /** To be able to assosiate between the features we keep the best found features from each camera. */
    int numRemovedFeaturesThisFrame;
    double relativeThresholdforUsingLiveFeatureScores_;
    double modalitySelectionAdditionScalar_;
    int minFeaturesPerModality_;


    // Temporary
    mutable PixelOutputCT pixelOutputCT_;
    mutable PixelOutput pixelOutput_;
    mutable MXD pixelOutputCov_;
    mutable rovtio::TransformFeatureOutputCT<mtState> transformFeatureOutputCT_;
    mutable FeatureOutput featureOutput_;
    mutable MXD featureOutputCov_;
    mutable MXD featureOutputJac_;
    mutable MultilevelPatch<mtState::nLevels_, mtState::patchSize_> mlpTemp1_;
    mutable MultilevelPatch<mtState::nLevels_, mtState::patchSize_> mlpTemp2_;
    mutable FeatureCoordinates alignedCoordinates_;
    mutable FeatureCoordinates tempCoordinates_;
    mutable FeatureCoordinatesVec candidates_;
    mutable cv::Point2f c_temp_;
    mutable Eigen::Matrix2d c_J_;
    mutable Eigen::Matrix2d A_red_;
    mutable Eigen::Vector2d b_red_;

    mutable Eigen::MatrixXd canditateGenerationH_;
    mutable Eigen::MatrixXd canditateGenerationDifVec_;
    mutable Eigen::MatrixXd canditateGenerationPy_;
    mutable Eigen::EigenSolver<Eigen::MatrixXd> candidateGenerationES_;

    mutable MultilevelPatchAlignment<mtState::nLevels_, mtState::patchSize_> alignment_; /**<Patch aligner*/
    mutable cv::Mat drawImg_; /**<Image currently used for drawing*/

    mutable std::fstream featureStatisticsFileOut_; // To collect the error distribution
    const bool collectFeatureStatistics = false;


    /** \brief Constructor.
     *
     *   Loads and sets the needed parameters.
     */
    ImgUpdate() : transformFeatureOutputCT_(nullptr),
                  pixelOutputCov_((int) (PixelOutput::D_), (int) (PixelOutput::D_)),
                  featureOutputCov_((int) (FeatureOutput::D_), (int) (FeatureOutput::D_)),
                  featureOutputJac_((int) (FeatureOutput::D_), (int) (mtState::D_)),
                  canditateGenerationH_(2, (int) (mtState::D_)),
                  canditateGenerationDifVec_((int) (mtState::D_), 1),
                  canditateGenerationPy_(2, 2) {
      mpMultiCamera_ = nullptr;
      initCovFeature_.setIdentity();
      initDepth_ = 0.5;
      startLevel_ = 3;
      endLevel_ = 1;
      startDetectionTh_ = 0.9;
      nDetectionBuckets_ = 100;
      scoreDetectionExponent_ = 0.5;
      penaltyDistance_ = 20;
      zeroDistancePenalty_ = nDetectionBuckets_ * 1.0;
      useDirectMethod_ = true;
      doFrameVisualisation_ = true;
      visualizePatches_ = false;
      verbose_ = false;
      trackingUpperBound_ = 0.9;
      trackingLowerBound_ = 0.1;
      minTrackedAndFreeFeatures_ = 0.5;
      minRelativeSTScore_ = 0.2;
      minTimeBetweenPatchUpdate_ = 1.0;
      removeNegativeFeatureAfterUpdate_ = true;
      doVisualMotionDetection_ = false;
      rateOfMovingFeaturesTh_ = 0.5;
      pixelCoordinateMotionTh_ = 1.0;
      minFeatureCountForNoMotionDetection_ = 5;
      minTimeForZeroVelocityUpdate_ = 1.0;
      maxUncertaintyToDepthRatioForDepthInitialization_ = 0.3;
      // Set update noise for different modalities
      for (int i = 0; i <mtState::nCam_; ++i){
        updateNoisePix_[i] = 2;
        updateNoiseInt_[i] = 400;
      }
      noiseGainForOffCamera_ = 4.0;
      alignConvergencePixelRange_ = 1.0;
      alignCoverageRatio_ = 2.0;
      alignMaxUniSample_ = 5;
      useCrossCameraMeasurements_ = true;
      doStereoInitialization_ = true;
      removalFactor_ = 1.1;
      alignmentGaussianWeightingSigma_ = 2.0;
      discriminativeSamplingDistance_ = 0.0;
      discriminativeSamplingGain_ = 0.0;
      maxAllowedFeatureDistance_ = 25.0;
      numRemovedFeaturesThisFrame = 0;

      relativeThresholdforUsingLiveFeatureScores_ = 0.5;
      modalitySelectionAdditionScalar_ = 1;
      maxDistanceForApproxamentlyLocalizingFeatures_ = -1.0;
      minFeaturesPerModality_ = 0;

      for (int i = 0;i<mtState::nCam_;++i){
        framesSinceLastDetectionAtempt_[i] = 0;
        minSTScoreForModalitySelection_[i] = 100;
        additionalScoreFromOtherModality_[i] = 50.0;
        fastDetectionThreshold_[i] = 10;
        minAbsoluteSTScore_[i] = 0.2;
        patchRejectionTh_[i] = 50.0;
        featureCountHistory_[i] = -1.0;
        previousUpdateOfFeatureCountHistory_[i] = -1.0;
      }

      doubleRegister_.registerDiagonalMatrix("initCovFeature", initCovFeature_);
      doubleRegister_.registerScalar("initDepth", initDepth_);
      doubleRegister_.registerScalar("startDetectionTh", startDetectionTh_);
      doubleRegister_.registerScalar("scoreDetectionExponent", scoreDetectionExponent_);
      doubleRegister_.registerScalar("penaltyDistance", penaltyDistance_);
      doubleRegister_.registerScalar("zeroDistancePenalty", zeroDistancePenalty_);
      doubleRegister_.registerScalar("trackingUpperBound", trackingUpperBound_);
      doubleRegister_.registerScalar("trackingLowerBound", trackingLowerBound_);
      doubleRegister_.registerScalar("minTrackedAndFreeFeatures", minTrackedAndFreeFeatures_);
      doubleRegister_.registerScalar("minRelativeSTScore", minRelativeSTScore_);
      doubleRegister_.registerScalar("minTimeBetweenPatchUpdate", minTimeBetweenPatchUpdate_);
      doubleRegister_.registerScalar("MotionDetection.rateOfMovingFeaturesTh", rateOfMovingFeaturesTh_);
      doubleRegister_.registerScalar("MotionDetection.pixelCoordinateMotionTh", pixelCoordinateMotionTh_);
      doubleRegister_.registerScalar("maxUncertaintyToDepthRatioForDepthInitialization",
                                     maxUncertaintyToDepthRatioForDepthInitialization_);
      doubleRegister_.registerScalar("alignConvergencePixelRange", alignConvergencePixelRange_);
      doubleRegister_.registerScalar("alignCoverageRatio", alignCoverageRatio_);
      doubleRegister_.registerScalar("removalFactor", removalFactor_);
      doubleRegister_.registerScalar("discriminativeSamplingDistance", discriminativeSamplingDistance_);
      doubleRegister_.registerScalar("discriminativeSamplingGain", discriminativeSamplingGain_);
      intRegister_.registerScalar("startLevel", startLevel_);
      intRegister_.registerScalar("endLevel", endLevel_);
      intRegister_.registerScalar("nDetectionBuckets", nDetectionBuckets_);
      intRegister_.registerScalar("MotionDetection.minFeatureCountForNoMotionDetection",
                                  minFeatureCountForNoMotionDetection_);
      intRegister_.registerScalar("alignMaxUniSample", alignMaxUniSample_);
      boolRegister_.registerScalar("MotionDetection.isEnabled", doVisualMotionDetection_);
      boolRegister_.registerScalar("useDirectMethod", useDirectMethod_);
      boolRegister_.registerScalar("doFrameVisualisation", doFrameVisualisation_);
      boolRegister_.registerScalar("visualizePatches", visualizePatches_);
      boolRegister_.registerScalar("removeNegativeFeatureAfterUpdate", removeNegativeFeatureAfterUpdate_);
      boolRegister_.registerScalar("useCrossCameraMeasurements", useCrossCameraMeasurements_);
      boolRegister_.registerScalar("doStereoInitialization", doStereoInitialization_);
      boolRegister_.registerScalar("useIntensityOffsetForAlignment", alignment_.useIntensityOffset_);
      boolRegister_.registerScalar("useIntensitySqewForAlignment", alignment_.useIntensitySqew_);
      doubleRegister_.removeScalarByVar(updnoiP_(0, 0));
      doubleRegister_.removeScalarByVar(updnoiP_(1, 1));
      for (int i = 0; i < mtState::nCam_; ++i) {
        doubleRegister_.registerScalar("UpdateNoise" + std::to_string(i) + ".pix", updateNoisePix_[i]);
        doubleRegister_.registerScalar("UpdateNoise" + std::to_string(i) + ".int", updateNoiseInt_[i]);
      }
      doubleRegister_.registerScalar("noiseGainForOffCamera", noiseGainForOffCamera_);
      useImprovedJacobian_ = false; // TODO: adapt/test
      isZeroVelocityUpdateEnabled_ = false;
      Base::PropertyHandler::registerSubHandler("ZeroVelocityUpdate", zeroVelocityUpdate_);
      zeroVelocityUpdate_.outlierDetection_.registerToPropertyHandler(&zeroVelocityUpdate_, "MahalanobisTh");
      zeroVelocityUpdate_.doubleRegister_.registerScalar("minNoMotionTime", minTimeForZeroVelocityUpdate_);
      zeroVelocityUpdate_.boolRegister_.registerScalar("isEnabled", isZeroVelocityUpdateEnabled_);
      doubleRegister_.removeScalarByStr("alpha");
      doubleRegister_.removeScalarByStr("beta");
      doubleRegister_.removeScalarByStr("kappa");
      alignmentHuberNormThreshold_ = static_cast<double>(alignment_.huberNormThreshold_);
      doubleRegister_.registerScalar("alignmentHuberNormThreshold", alignmentHuberNormThreshold_);
      doubleRegister_.registerScalar("alignmentGaussianWeightingSigma", alignmentGaussianWeightingSigma_);
      alignmentGradientExponent_ = static_cast<double>(alignment_.gradientExponent_);
      doubleRegister_.registerScalar("alignmentGradientExponent", alignmentGradientExponent_);
      doubleRegister_.registerScalar("maxAllowedFeatureDistance", maxAllowedFeatureDistance_);
      doubleRegister_.registerScalar("maxDistanceForApproximatlyLocalizingFeatures", maxDistanceForApproxamentlyLocalizingFeatures_);
      doubleRegister_.registerScalar("relativeThresholdforUsingLiveFeatureScores", relativeThresholdforUsingLiveFeatureScores_);
      doubleRegister_.registerScalar("modalitySelectionAdditionScalar", modalitySelectionAdditionScalar_);
      intRegister_.registerScalar("minFeaturesPerModality", minFeaturesPerModality_);
      for (int i = 0; i < mtState::nCam_; ++i) {
        doubleRegister_.registerScalar("minSTScoreForModalitySelection" + std::to_string(i), minSTScoreForModalitySelection_[i]);
        doubleRegister_.registerScalar("additionalScoreFromOtherModality" + std::to_string(i), additionalScoreFromOtherModality_[i]);
        doubleRegister_.registerScalar("minAbsoluteSTScore" + std::to_string(i), minAbsoluteSTScore_[i]); // Custom moved into this loop
        intRegister_.registerScalar("fastDetectionThreshold" + std::to_string(i), fastDetectionThreshold_[i]); // Custom moved into this loop
        doubleRegister_.registerScalar("patchRejectionTh" + std::to_string(i), patchRejectionTh_[i]);
      }

      if (collectFeatureStatistics){
        featureStatisticsFileOut_.open("featureStatistics_rovtio.csv", std::ios::out);
        featureStatisticsFileOut_.precision(17);
        featureStatisticsFileOut_ << std::fixed;
        featureStatisticsFileOut_ << "time" << ", " << "camera" << ", " << "#active features" << ", " << "#added features this frame" << ", " << "#removed features this frame" << ", " << "average feature lifetime" << "\n";
      }
    };

    /** \brief Destructor
     */
    virtual ~ImgUpdate() {};

    /** \brief Refresh the properties of the property handler
     */
    void refreshProperties() {
      if (isZeroVelocityUpdateEnabled_) assert(doVisualMotionDetection_);
      for (int camID = 0; camID < mtState::nCam_; ++camID) {
        if (useDirectMethod_) {
          updnoiP_.setIdentity();
          updnoiP_ = updnoiP_ * updateNoiseInt_[camID];
        } else {
          updnoiP_.setIdentity();
          updnoiP_ = updnoiP_ * updateNoisePix_[camID];
        }
      }
      alignment_.huberNormThreshold_ = static_cast<float>(alignmentHuberNormThreshold_);
      alignment_.computeWeightings(alignmentGaussianWeightingSigma_);
      alignment_.gradientExponent_ = static_cast<float>(alignmentGradientExponent_);
    };

    /** \brief Sets the multicamera pointer
     *
     * @param mpMultiCamera - Multicamera pointer
     */
    void setCamera(MultiCamera<mtState::nCam_> *mpMultiCamera) {
      mpMultiCamera_ = mpMultiCamera;
      transformFeatureOutputCT_.mpMultiCamera_ = mpMultiCamera;
    }

    /** \brief Sets the innovation term.
     *
     *  \note If \ref useDirectMethod_ is set true, the innovation term is based directly on pixel intensity errors.
     *  \note If \ref useDirectMethod_ is set false, the reprojection error is used for the innovation term.
     *  @param mtInnovation - Class, holding innovation data.
     *  @param state        - Filter %State.
     *  @param noise        - Additive discrete Gaussian noise.
     */
    void evalInnovation(mtInnovation &y, const mtState &state, const mtNoise &noise) const {
      const int &ID = state.aux().activeFeature_;
      const int &camID = state.CfP(ID).camID_;
      const int activeCamID = (state.aux().activeCameraCounter_ + camID) % mtState::nCam_;
      transformFeatureOutputCT_.setFeatureID(ID);
      transformFeatureOutputCT_.setOutputCameraID(activeCamID);
      transformFeatureOutputCT_.transformState(state, featureOutput_);

      if (meas_.aux().activeModality_ == activeCamID) {
        if (useDirectMethod_) {
          if (doFrameVisualisation_ && featureOutput_.c().com_c()) {
            if (activeCamID == camID) {
              featureOutput_.c().drawPoint(drawImg_, cv::Scalar(0, 175, 175));
            } else {
              featureOutput_.c().drawPoint(drawImg_, cv::Scalar(175, 175, 0));
            }
          }
          if (alignment_.getLinearAlignEquationsReduced(meas_.aux().pyr_[activeCamID],
                                                        *state.aux().mpCurrentFeature_->mpMultilevelPatch_,
                                                        featureOutput_.c(), endLevel_, startLevel_, A_red_, b_red_)) {
            y.template get<mtInnovation::_pix>() = b_red_ + noise.template get<mtNoise::_pix>();
            if (verbose_) {
              std::cout << "    \033[32mMaking update with feature " << ID << " from camera " << camID
                        << " in camera " << activeCamID << "\033[0m" << std::endl;
            }
          } else {
            y.template get<mtInnovation::_pix>() = noise.template get<mtNoise::_pix>();
            if (verbose_) {
              std::cout << "    \033[31mFailed Construction of Alignment Equations with feature " << ID
                        << " from camera " << camID << " in camera " << activeCamID << "\033[0m" << std::endl;
            }
            cancelIteration_ = true;
          }
        } else {
          Eigen::Vector2d pixError;
          pixError(0) = static_cast<double>(state.aux().feaCoorMeas_[ID].get_c().x - featureOutput_.c().get_c().x);
          pixError(1) = static_cast<double>(state.aux().feaCoorMeas_[ID].get_c().y - featureOutput_.c().get_c().y);
          y.template get<mtInnovation::_pix>() = pixError + noise.template get<mtNoise::_pix>();
        }
      }
    }

    bool generateCandidates(const mtFilterState &filterState, mtState &candidate) const {
      candidate = filterState.state_;

      if (candidateCounter_ == 0) {
        const int &ID = candidate.aux().activeFeature_;
        const int &camID = candidate.CfP(ID).camID_;
        const int activeCamID = (candidate.aux().activeCameraCounter_ + camID) % mtState::nCam_;

        transformFeatureOutputCT_.setFeatureID(ID);
        transformFeatureOutputCT_.setOutputCameraID(activeCamID);
        transformFeatureOutputCT_.transformState(candidate, featureOutput_);
        transformFeatureOutputCT_.jacTransform(featureOutputJac_, candidate);
        mpMultiCamera_->cameras_[activeCamID].bearingToPixel(featureOutput_.c().get_nor(), c_temp_, c_J_);

        canditateGenerationH_ = -c_J_ * featureOutputJac_.template block<2, mtState::D_>(0, 0);
        canditateGenerationPy_ = canditateGenerationH_ * filterState.cov_ * canditateGenerationH_.transpose();
        candidateGenerationES_.compute(canditateGenerationPy_);
      }

      while (++candidateCounter_) {
        int u = (candidateCounter_ - 1) / (2 * alignMaxUniSample_ + 1) - alignMaxUniSample_;
        if (u > alignMaxUniSample_)
          break;
        int v = (candidateCounter_ - 1) % (2 * alignMaxUniSample_ + 1) - alignMaxUniSample_;
        if (pow(u * alignConvergencePixelRange_, 2) / candidateGenerationES_.eigenvalues()(0).real()
            + pow(v * alignConvergencePixelRange_, 2) / candidateGenerationES_.eigenvalues()(1).real() <
            pow(alignCoverageRatio_, 2)) {
          Eigen::Vector2d dy = u * alignConvergencePixelRange_ * candidateGenerationES_.eigenvectors().col(0).real()
                               + v * alignConvergencePixelRange_ * candidateGenerationES_.eigenvectors().col(1).real();
          canditateGenerationDifVec_ =
                  -filterState.cov_ * canditateGenerationH_.transpose() * canditateGenerationPy_.inverse() * dy;
          candidate.boxPlus(canditateGenerationDifVec_, candidate);
          return true;
        }
      }
      return false;
    }

    bool extraOutlierCheck(const mtState &state) const {
      const int &ID = state.aux().activeFeature_;
      const int &camID = state.CfP(ID).camID_;
      const int activeCamID = (state.aux().activeCameraCounter_ + camID) % mtState::nCam_;
      transformFeatureOutputCT_.setFeatureID(ID);
      transformFeatureOutputCT_.setOutputCameraID(activeCamID);
      transformFeatureOutputCT_.transformState(state, featureOutput_);

      if (!hasConverged_) {
        if (verbose_) std::cout << "    \033[31mREJECTED (iterations did not converge)\033[0m" << std::endl;
        if (mlpTemp1_.isMultilevelPatchInFrame(meas_.aux().pyr_[activeCamID], featureOutput_.c(), startLevel_, false)) {
          featureOutput_.c().drawPoint(drawImg_, cv::Scalar(255, 0, 0), 1.0);
        }
        return false;
      }

      if (patchRejectionTh_[camID] >= 0) {
        if (!mlpTemp1_.isMultilevelPatchInFrame(meas_.aux().pyr_[activeCamID], featureOutput_.c(), startLevel_,
                                                false)) {
          if (verbose_) std::cout << "    \033[31mREJECTED (not in frame)\033[0m" << std::endl;
          return false;
        }
        mlpTemp1_.extractMultilevelPatchFromImage(meas_.aux().pyr_[activeCamID], featureOutput_.c(), startLevel_,
                                                  false);
        const float avgError = mlpTemp1_.computeAverageDifference(*state.aux().mpCurrentFeature_->mpMultilevelPatch_,
                                                                  endLevel_, startLevel_);
        if (avgError > patchRejectionTh_[camID]) {
          if (verbose_) std::cout << "    \033[31mREJECTED (error too large: " << avgError << ")\033[0m" << std::endl;
          featureOutput_.c().drawPoint(drawImg_, cv::Scalar(255, 255, 0), 1.0);
          return false;
        }

        // Use 4 sample around feature, at least two should be above the treshold
        if (discriminativeSamplingDistance_ > 0.0) {
          FeatureOutput sample;
          V3D d;
          int countAboveThreshold = 0;
          for (int i = 0; i < 4; i++) {
            d.setZero();
            d(i % 2) = (i / 2 * 2 - 1) * discriminativeSamplingDistance_;
            featureOutput_.boxPlus(d, sample);
            if (mlpTemp1_.isMultilevelPatchInFrame(meas_.aux().pyr_[activeCamID], sample.c(), startLevel_, false)) {
              mlpTemp1_.extractMultilevelPatchFromImage(meas_.aux().pyr_[activeCamID], sample.c(), startLevel_, false);
              const float sampleError = mlpTemp1_.computeAverageDifference(
                      *state.aux().mpCurrentFeature_->mpMultilevelPatch_, endLevel_, startLevel_);
              const bool isAboveThreshold = (discriminativeSamplingGain_ <= 1.0 & sampleError > patchRejectionTh_[camID])
                                            | (discriminativeSamplingGain_ > 1.0 &
                                               sampleError > discriminativeSamplingGain_ * avgError);
              countAboveThreshold += isAboveThreshold;
              if (isAboveThreshold) {
                sample.c().drawPoint(drawImg_, cv::Scalar(0, 255, 0), 2.0);
              } else {
                sample.c().drawPoint(drawImg_, cv::Scalar(0, 0, 255), 2.0);
              }
            }
          }
          if (countAboveThreshold < 2) {
            if (verbose_)
              std::cout << "    \033[31mREJECTED (feature location not discriminative enough)\033[0m" << std::endl;
            return false;
          }
        }
      }

      featureOutput_.c().drawPoint(drawImg_, cv::Scalar(0, 0, 255), 1.0);
      return true;
    }

    /** \brief Computes the Jacobian for the update step of the filter.
     *
     *  \note If \ref useDirectMethod_ is set true, the jacobian is set w.r.t. the intensity errors.
     *  \note If \ref useDirectMethod_ is set false, the jacobian is set w.r.t. the reprojection error.
     *  @param F     - Jacobian for the update step of the filter.
     *  @param state - Filter state.
     */
    void jacState(MXD &F, const mtState &state) const {
      const int &ID = state.aux().activeFeature_;
      const int &camID = state.CfP(ID).camID_;
      const int activeCamID = (state.aux().activeCameraCounter_ + camID) % mtState::nCam_;
      transformFeatureOutputCT_.setFeatureID(ID);
      transformFeatureOutputCT_.setOutputCameraID(activeCamID);
      transformFeatureOutputCT_.transformState(state, featureOutput_);

      if (useDirectMethod_) {
        if (alignment_.getLinearAlignEquationsReduced(meas_.aux().pyr_[activeCamID],
                                                      *state.aux().mpCurrentFeature_->mpMultilevelPatch_,
                                                      featureOutput_.c(), endLevel_, startLevel_, A_red_, b_red_)) {
          transformFeatureOutputCT_.jacTransform(featureOutputJac_, state);
          mpMultiCamera_->cameras_[activeCamID].bearingToPixel(featureOutput_.c().get_nor(), c_temp_, c_J_);
          F = -A_red_ * c_J_ * featureOutputJac_.template block<2, mtState::D_>(0, 0);
        } else {
          F.setZero();
          cancelIteration_ = true;
        }
      } else {
        transformFeatureOutputCT_.jacTransform(featureOutputJac_, state);
        mpMultiCamera_->cameras_[activeCamID].bearingToPixel(featureOutput_.c().get_nor(), c_temp_, c_J_);
        F = -c_J_ * featureOutputJac_.template block<2, mtState::D_>(0, 0);
      }
    }

    /** \brief Computes the Jacobian for the update step of the filter.
     *
     *  @param G     - Jacobian for the update step of the filter.
     *  @param state - Filter state.
     */
    void jacNoise(MXD &G, const mtState &state) const {
      G.setZero();
      G.template block<2, 2>(mtInnovation::template getId<mtInnovation::_pix>(),
                             mtNoise::template getId<mtNoise::_pix>()) = Eigen::Matrix2d::Identity();
    }

    /** \brief Prepares the filter state for the update.
     *
     *   @param filterState - Filter state.
     *   @param meas        - Update measurement.
     *   @todo sort feature by covariance and use more accurate ones first
     */
    void commonPreProcess(mtFilterState &filterState, const mtMeas &meas) {
      assert(filterState.t_ == meas.aux().imgTime_);

      if (doFrameVisualisation_) {
        //Check if image needs to be normalized for display purposes
        double imgMin, imgMax;
        cv::minMaxLoc(meas.aux().pyr_[meas.aux().activeModality_].imgs_[startLevel_], &imgMin,
                      &imgMax); //Check on the highest allowed pyramid level to save time, Equaliztion applied on original image
        cv::Mat tmpImg;
        if (imgMax > 255.0)
          meas.aux().pyr_[meas.aux().activeModality_].equalizeHistogram(0, tmpImg);
        else
          meas.aux().pyr_[meas.aux().activeModality_].imgs_[0].convertTo(tmpImg, CV_8UC1);
        //Convert Image to color for drawing purposes
        cvtColor(tmpImg, filterState.img_[meas.aux().activeModality_], CV_GRAY2RGB);
      }
      filterState.imgTime_ = filterState.t_;
      filterState.imageCounter_++;
      if (visualizePatches_) {
        filterState.patchDrawing_ = cv::Mat::zeros(mtState::nMax_ * filterState.drawPS_,
                                                   (1 + 2 * mtState::nCam_) * filterState.drawPS_, CV_32FC3);
      }
      filterState.state_.aux().activeFeature_ = 0;
      filterState.state_.aux().activeCameraCounter_ = 0;


      /* Detect Image changes by looking at the feature patches between current and previous image (both at the current feature location)
       * The maximum change of intensity is obtained if the pixel is moved along the strongest gradient.
       * The maximal singularvalue, which is equivalent to the root of the larger eigenvalue of the Hessian,
       * gives us range in which intensity change is allowed to be.
       */
      if (doVisualMotionDetection_ && filterState.imageCounter_ > 1) {
        int totCountInFrame = 0;
        int totCountInMotion = 0;
        for (unsigned int i = 0; i < mtState::nMax_; i++) {
          if (filterState.fsm_.isValid_[i]) {
            const int &camID = filterState.state_.CfP(i).camID_;   // Camera ID of the feature.
            if (camID == meas.aux().activeModality_) { // Custom added check to only work on the image which is present
              tempCoordinates_ = *filterState.fsm_.features_[i].mpCoordinates_;
              tempCoordinates_.set_warp_identity();
              if (mlpTemp1_.isMultilevelPatchInFrame(filterState.prevPyr_[camID], tempCoordinates_, startLevel_,
                                                     true)) {
                mlpTemp1_.extractMultilevelPatchFromImage(filterState.prevPyr_[camID], tempCoordinates_, startLevel_,
                                                          true);
                mlpTemp1_.computeMultilevelShiTomasiScore(endLevel_, startLevel_);
                mlpTemp2_.extractMultilevelPatchFromImage(meas.aux().pyr_[camID], tempCoordinates_, startLevel_, true);
                const float avgError = mlpTemp1_.computeAverageDifference(mlpTemp2_, endLevel_, startLevel_);
                if (avgError / std::sqrt(mlpTemp1_.e1_) > static_cast<float>(pixelCoordinateMotionTh_))
                  totCountInMotion++;
                totCountInFrame++;
              }
            }
          }
        }
        if (rateOfMovingFeaturesTh_ / totCountInMotion * totCountInFrame < 1.0 ||
            totCountInFrame < minFeatureCountForNoMotionDetection_) {
          filterState.state_.aux().timeSinceLastImageMotion_ = 0.0;
        }
      }
      numRemovedFeaturesThisFrame = 0;
    }

    /** \brief Pre-Processing for the image update.
     *
     *  Summary:
     *  1. Searches a valid MultilevelPatchFeature from the filter state.
     *  2. Transforms the MultilevelPatchFeature into the target frame.
     *  3. Executes a 2D patch alignment in the target frame. If unsuccessful go back to step 1.
     *  4. If bearing error between aligned patch and estimated patch too large, alter linearization point.
     *     Bearing vector in the state is directly altered to the new aligned position.
     *
     *  @param filterState - Filter state.
     *  @param meas        - Update measurement.
     *  @param isFinished  - True, if process has finished.
     *  @todo split into methods
     */
    void preProcess(mtFilterState &filterState, const mtMeas &meas, bool &isFinished) {
      if (isFinished) { // gets called if this is the first call
        commonPreProcess(filterState, meas);
        isFinished = false;
      }

      bool foundValidMeasurement = false;
      typename mtFilterState::mtState &state = filterState.state_;
      MXD &cov = filterState.cov_;
      int &ID = filterState.state_.aux().activeFeature_;   // ID of the current updated feature!!! Initially set to 0.
      int &activeCamCounter = filterState.state_.aux().activeCameraCounter_;

      // Actualize camera extrinsics (gets also update in calls to TransformFeatureOutputCT)
      state.updateMultiCameraExtrinsics(mpMultiCamera_);

      while (ID < mtState::nMax_ && foundValidMeasurement == false) {
        if (filterState.fsm_.isValid_[ID]) {
          // Data handling stuff
          FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_> &f = filterState.fsm_.features_[ID];
          const int camID = f.mpCoordinates_->camID_;
          const int activeCamID = (activeCamCounter + camID) % mtState::nCam_;
          if (camID != activeCamID)
            std::cout << "camIds are not equal: camID: " << camID << "activeCamID: " << activeCamID
                      << ", activeCamCounter: " << activeCamCounter << '\n';

          if (camID == meas.aux().activeModality_) {

            drawImg_ = filterState.img_[activeCamID];
            if (activeCamCounter == 0) {
              f.mpStatistics_->increaseStatistics(filterState.t_);
              if (verbose_) {
                std::cout << "=========== Feature " << f.idx_
                          << " ==================================================== "
                          << std::endl;
              }
              // Visualize patch tracking
              if (visualizePatches_) {
                f.mpMultilevelPatch_->drawMultilevelPatch(filterState.patchDrawing_,
                                                          cv::Point2i(2,
                                                                      filterState.drawPB_ + ID * filterState.drawPS_),
                                                          1, false);
                cv::putText(filterState.patchDrawing_, std::to_string(f.idx_),
                            cv::Point2i(2, 10 + ID * filterState.drawPS_), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                            cv::Scalar(255, 255, 255));
              }
              f.log_prediction_ = *f.mpCoordinates_;
            }
            if (verbose_) {
              std::cout << "  ========== Camera  " << activeCamID << " ================= " << std::endl;
              std::cout << "  Normal in feature frame: " << f.mpCoordinates_->get_nor().getVec().transpose()
                        << std::endl;
              std::cout << "  with depth: " << f.mpDistance_->getDistance() << std::endl;
            }
          }

          // Get coordinates in target frame
          transformFeatureOutputCT_.setFeatureID(ID);
          transformFeatureOutputCT_.setOutputCameraID(activeCamID);
          transformFeatureOutputCT_.transformState(state, featureOutput_);
          transformFeatureOutputCT_.transformCovMat(state, cov, featureOutputCov_);
          if (verbose_)
            std::cout << "    Normal in camera frame: " << featureOutput_.c().get_nor().getVec().transpose()
                      << std::endl;

          if (camID == meas.aux().activeModality_) {   // Does not find patches in frames which are not updated

            // Check if feature in target frame
            if (!mlpTemp1_.isMultilevelPatchInFrame(filterState.prevPyr_[camID], featureOutput_.c(), startLevel_,
                                                    false)) {
              f.mpStatistics_->status_[activeCamID] = NOT_IN_FRAME;
              if (verbose_) std::cout << "    NOT in frame" << std::endl;
            } else {
              pixelOutputCT_.transformState(featureOutput_, pixelOutput_);
              pixelOutputCT_.transformCovMat(featureOutput_, featureOutputCov_, pixelOutputCov_);
              featureOutput_.c().setPixelCov(pixelOutputCov_);

              // Visualization
              if (doFrameVisualisation_) {
                if (activeCamID == camID) {
                  featureOutput_.c().drawEllipse(drawImg_, cv::Scalar(0, 175, 175), 2.0, true);
                  featureOutput_.c().drawText(drawImg_, std::to_string(f.idx_), cv::Scalar(0, 175, 175));
                } else {
                  featureOutput_.c().drawEllipse(drawImg_, cv::Scalar(175, 175, 0), 2.0, true);
                  featureOutput_.c().drawText(drawImg_, std::to_string(f.idx_), cv::Scalar(175, 175, 0));
                }
              }
              if (visualizePatches_) {
                if (mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[activeCamID], featureOutput_.c(), startLevel_,
                                                       false)) {
                  mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[activeCamID], featureOutput_.c(),
                                                            startLevel_,
                                                            false);
                  mlpTemp1_.drawMultilevelPatch(filterState.patchDrawing_, cv::Point2i(
                          filterState.drawPB_ + (1 + 2 * activeCamID) * filterState.drawPS_,
                          filterState.drawPB_ + ID * filterState.drawPS_), 1, false);
                }
              }
              if (useDirectMethod_) {
                if (verbose_) std::cout << "    Do direct update (without alignment)" << std::endl;
                state.aux().mpCurrentFeature_ = &filterState.fsm_.features_[ID];
                if (activeCamCounter == 0) {
                  updnoiP_.setIdentity();
                  updnoiP_ = updnoiP_ * updateNoiseInt_[camID];
                } else {
                  updnoiP_.setIdentity();
                  updnoiP_ = updnoiP_ * updateNoiseInt_[camID] * noiseGainForOffCamera_;
                }
                foundValidMeasurement = true;
              } else {
                if (alignment_.align2DAdaptive(alignedCoordinates_, meas.aux().pyr_[activeCamID], *f.mpMultilevelPatch_,
                                               featureOutput_.c(), startLevel_, endLevel_,
                                               alignConvergencePixelRange_, alignCoverageRatio_, alignMaxUniSample_)) {
                  if (verbose_)
                    std::cout << "    Found match: " << alignedCoordinates_.get_nor().getVec().transpose() << std::endl;
                  if (mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[activeCamID], alignedCoordinates_, startLevel_,
                                                         false)) {
                    float avgError = 0.0;
                    if (patchRejectionTh_[camID] >= 0) {
                      mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[activeCamID], alignedCoordinates_,
                                                                startLevel_, false);
                      avgError = mlpTemp1_.computeAverageDifference(*f.mpMultilevelPatch_, endLevel_, startLevel_);
                    }
                    if (patchRejectionTh_[camID] >= 0 && avgError > patchRejectionTh_[camID]) {
                      f.mpStatistics_->status_[activeCamID] = FAILED_ALIGNEMENT;
                      if (verbose_) std::cout << "    \033[31mREJECTED (error too large)\033[0m" << std::endl;
                    } else {
                      if (doFrameVisualisation_) alignedCoordinates_.drawPoint(drawImg_, cv::Scalar(255, 0, 255));
                      state.aux().feaCoorMeas_[ID] = alignedCoordinates_;
                      foundValidMeasurement = true;
                    }
                  } else {
                    f.mpStatistics_->status_[activeCamID] = FAILED_ALIGNEMENT;
                    if (verbose_) std::cout << "    \033[31mNot in frame after alignment\033[0m" << std::endl;
                  }
                } else {
                  f.mpStatistics_->status_[activeCamID] = FAILED_ALIGNEMENT;
                  if (verbose_) std::cout << "    \033[31mNOT FOUND (matching failed)\033[0m" << std::endl;
                }
              }
            }
          }
        }
        if (foundValidMeasurement == false) {
          activeCamCounter++;
          if (activeCamCounter == mtState::nCam_ || !useCrossCameraMeasurements_) {
            activeCamCounter = 0;
            ID++;
          }
        }
      }  // while end
      if (ID >= mtState::nMax_) {
        isFinished = true;
      }
    };

    /** \brief Restricts the number of found features if this modality is infeasible to track
     * // TODO: Make this inline
     */
    int maxFeaturesFromThisModality(const mtFilterState &filterState, const mtMeas &meas, const int camID,
                                    std::array<int, mtState::nCam_> numFound, float deadFeaturesLifetime,
                                    double timeSinceLastFeatureDied,
                                    std::array<double, mtState::nCam_> &featureCountHistoryScore,
                                    const double featureCountHistoryLength, const double timeSinceLastCall) {
      if (mtState::nCam_ <= 1) {
        return mtState::nMax_;
      }

      int inactiveCameras = 0;
      for (int i = 0; i < mtState::nCam_; ++i) {
        if (meas.aux().camActive_[i] == false){
          inactiveCameras++;
        }
      }

      std::array<int, mtState::nCam_> featuresPerCam = {0};
      std::array<double, mtState::nCam_> averageFeatureLifetime = {0.0};
      for (int i = 0;i<mtState::nMax_; ++i){
        if (filterState.fsm_.isValid_[i] ){
          const int featureModality = filterState.fsm_.features_[i].mpCoordinates_->camID_;
          ++featuresPerCam[featureModality];
          const FeatureStatistics<mtState::nCam_> featureStatistics = *filterState.fsm_.features_[i].mpStatistics_;
          averageFeatureLifetime[featureModality] +=   featureStatistics.currentTime_ - featureStatistics.initTime_;
        }
      }
      double totalAverageLifetimes = 0;
      for (int i = 0; i < mtState::nCam_; ++i) {
        averageFeatureLifetime[i] = featuresPerCam[i] == 0 ? 0 : averageFeatureLifetime[i]/(double)featuresPerCam[i];
        totalAverageLifetimes += averageFeatureLifetime[i];
      }

      const int maxFeaturesToAdd = mtState::nMax_ - featuresPerCam[meas.aux().activeModality_] - minFeaturesPerModality_ * std::max(mtState::nCam_-1 - inactiveCameras, 0);
      const int minFeaturesToAdd = std::max(minFeaturesPerModality_ - featuresPerCam[meas.aux().activeModality_], 0);
      const int possibleToAdd = mtState::nMax_ - filterState.fsm_.getValidCount();

      std::cout << "camID:" << camID << ", numFound[0]: " << numFound[0] << ", numfound[1]:" << numFound[1] << '\n';


      int totalFound = 0;
      for (int i = 0; i < mtState::nCam_;++i) {
        if (meas.aux().camActive_[i]) {
          totalFound += numFound[i];
        }
      }
      if (totalFound <= 0) {
        totalFound = 1;
      }

      // Increment featurecounthistoryscore
      std::array<double, mtState::nCam_> inclusionFactors;
      inclusionFactors.fill(timeSinceLastCall/featureCountHistoryLength);
      if (featureCountHistoryScore[camID] < 0) {
        featureCountHistoryScore[camID] = 0.5;
      } else {
        const double newIncrement =  (double) numFound[camID] / (double) totalFound;

        if (verbose_) std::cout << "Old feathrescore: " << featureCountHistoryScore[camID] << ", new increment: " << newIncrement << "inclusionfactor: " << inclusionFactors[camID] << ", timeSinceLastCall: " << timeSinceLastCall << '\n';
        featureCountHistoryScore[camID] =
                ((1.0 - inclusionFactors[camID]) * featureCountHistoryScore[camID]) +
                (inclusionFactors[camID] * newIncrement);
      }


      std::cout << "Number of features for this cam: " << numFound[camID] << ", totalFound: " << totalFound << ", maxFeaturesToAdd:" << maxFeaturesToAdd << ", possibleToAdd:" << possibleToAdd << ", features[0]" << featuresPerCam[0] << ", features[1]:" << featuresPerCam[1] << ", averageFeaturesLifetime[0]: " << averageFeatureLifetime[0] << ", averageFeatureLifetime[1]: " << averageFeatureLifetime[1]  <<'\n';
      const double singleShotScore = (double)numFound[camID] / (double)totalFound; // A score considering only how much of this spesific image that is suited for tracking.
      const double liveFeatureLiftimeScore = averageFeatureLifetime[camID] / totalAverageLifetimes; // A score considering how good a modality have traditionally been.
      const double historicalFeatureCountScore = featureCountHistoryScore[camID];
      double resScore = 0.0;
      const double neutralScore = 1.0 / (double) mtState::nCam_;
      if (totalFound < possibleToAdd/2){
        // Both modalities are very poor
        if (verbose_) std::cout << "Very few features found, taking all we can get. ";
        resScore = 1.0;
      } else{
        if (averageFeatureLifetime[camID] <= 1e-17 || (historicalFeatureCountScore > 0 && (historicalFeatureCountScore > neutralScore * (1+ relativeThresholdforUsingLiveFeatureScores_) || historicalFeatureCountScore < neutralScore * relativeThresholdforUsingLiveFeatureScores_))){
          // There is a significant difference in how good the two modalities have been.
          resScore = historicalFeatureCountScore * historicalFeatureCountScore * singleShotScore * mtState::nCam_ * mtState::nCam_;
        }else{
          resScore = liveFeatureLiftimeScore * liveFeatureLiftimeScore * historicalFeatureCountScore * historicalFeatureCountScore * singleShotScore * mtState::nCam_ * mtState::nCam_ * mtState::nCam_ * mtState::nCam_;
        }
      }
      std::cout << "Verbose:" << verbose_ << '\n';
      if (verbose_) std::cout << "Singleshotscore: " << singleShotScore << ", historicalfeaturecountscore: " << historicalFeatureCountScore << ", livefeaturelifetimescore: " << liveFeatureLiftimeScore << ", resScore: " << resScore << '\n';
      int res = (int) (resScore * modalitySelectionAdditionScalar_ * (double) (mtState::nMax_)) - featuresPerCam[meas.aux().activeModality_];
      res = res > maxFeaturesToAdd ? maxFeaturesToAdd : res;
      res = res < minFeaturesToAdd ? minFeaturesToAdd : res;
      if(verbose_) std::cout << "RelativeScore:" << singleShotScore << ", resulting features: " << res << '\n';
      return res;
    }


    /** \brief Post-Processing for the image update.
     *
     *  Summary:
     *  1. Some drawing tasks.
     *  2. Removal of features with negative depth.
     *
     *  @param filterState      - Filter state.
     *  @param meas             - Update measurement.
     *  @param outlierDetection - Outlier detection.
     *  @param isFinished       - True, if process has finished.
     */
    void postProcess(mtFilterState &filterState, const mtMeas &meas, const mtOutlierDetection &outlierDetection,
                     bool &isFinished) {
      int &ID = filterState.state_.aux().activeFeature_;  // Get the ID of the updated feature.
      int &activeCamCounter = filterState.state_.aux().activeCameraCounter_;



      if (isFinished) {
        commonPostProcess(filterState, meas);
      } else {
        FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_> &f = filterState.fsm_.features_[ID];
        const int camID = f.mpCoordinates_->camID_;
        const int activeCamID = (activeCamCounter + camID) % mtState::nCam_;

        // Remove negative feature
        if (removeNegativeFeatureAfterUpdate_) {
          for (unsigned int i = 0; i < mtState::nMax_; i++) {
            if (filterState.fsm_.isValid_[i]) {
              if (filterState.state_.dep(i).getDistance() < 1e-8 ||
                  ((filterState.fsm_.features_[i].mpStatistics_->currentTime_ -
                    filterState.fsm_.features_[i].mpStatistics_->initTime_) > 1.0 &&
                   filterState.state_.dep(i).getDistance() >
                   maxAllowedFeatureDistance_))
              {
                if (verbose_)
                  std::cout << "    \033[33mRemoved feature " << filterState.fsm_.features_[i].idx_
                            << " with invalid distance parameter " << filterState.state_.dep(i).p_ << "!\033[0m"
                            << std::endl;
                filterState.fsm_.isValid_[i] = false;
                numRemovedFeaturesThisFrame += 1;
                filterState.resetFeatureCovariance(i, Eigen::Matrix3d::Identity());
              }
            }
          }
        }

        if (filterState.fsm_.isValid_[ID]) {
          // Update status and visualization
          transformFeatureOutputCT_.setFeatureID(ID);
          transformFeatureOutputCT_.setOutputCameraID(activeCamID);
          transformFeatureOutputCT_.transformState(filterState.state_, featureOutput_);

          // Draw information ellipse
          bool doInformationGainVizualization = false;
          if (doFrameVisualisation_ && doInformationGainVizualization) {
            MXD F(2, 2);
            F = A_red_;
            F = F.transpose() * F * 1.0 / updateNoiseInt_[camID]; // Custom: Use the one for this camera
            featureOutput_.c().setPixelCov(F);
            featureOutput_.c().drawEllipse(drawImg_, cv::Scalar(0, 255, 0), 10, false);
            F.setIdentity();
            F = F.transpose() * F * 1.0 / updateNoisePix_[camID]; // Custom: Use the one for this camera
            featureOutput_.c().setPixelCov(F);
            featureOutput_.c().drawEllipse(drawImg_, cv::Scalar(0, 0, 255), 10, false);
          }
          filterState.mlpErrorLog_[ID] = alignment_.mlpError_;

          if ((filterState.mode_ == LWF::ModeIEKF && successfulUpdate_) ||
              (filterState.mode_ == LWF::ModeEKF && !outlierDetection.isOutlier(0))) {
            if (mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[camID], featureOutput_.c(), startLevel_, false)) {
              f.mpStatistics_->status_[activeCamID] = TRACKED;
              if (doFrameVisualisation_)
                mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0, cv::Scalar(0, 150 +
                                                                                                     (activeCamID ==
                                                                                                      camID) * 105, 0));
            } else {
              f.mpStatistics_->status_[activeCamID] = FAILED_TRACKING;
              if (doFrameVisualisation_) {
                mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0,
                                                    cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
                featureOutput_.c().drawText(drawImg_, "NIF", cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
              }
              if (verbose_) std::cout << "    \033[31mNot in frame after update!\033[0m" << std::endl;
            }
          } else {
            f.mpStatistics_->status_[activeCamID] = FAILED_TRACKING;
            if (outlierDetection.isOutlier(0)) {
              if (doFrameVisualisation_) {
                mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0,
                                                    cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
                featureOutput_.c().drawText(drawImg_, "MD: " + std::to_string(outlierDetection.getMahalDistance(0)),
                                            cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
              }
              if (verbose_)
                std::cout << "    \033[31mRecognized as outlier by filter: " << outlierDetection.getMahalDistance(0)
                          << "\033[0m" << std::endl;
            } else if (!hasConverged_) {
              if (doFrameVisualisation_) {
                mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0,
                                                    cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
                featureOutput_.c().drawText(drawImg_, "INC", cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
              }
              if (verbose_) std::cout << "    \033[31mIterations not converged!\033[0m" << std::endl;
            } else {
              if (doFrameVisualisation_) {
                mlpTemp1_.drawMultilevelPatchBorder(drawImg_, featureOutput_.c(), 1.0,
                                                    cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
                featureOutput_.c().drawText(drawImg_, "PE", cv::Scalar(0, 0, 150 + (activeCamID == camID) * 105));
              }
              if (verbose_) std::cout << "    \033[31mToo large pixel intesity error!\033[0m" << std::endl;
            }
          }

          // Visualize patch tracking
          if (visualizePatches_) {
            if (mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[activeCamID], featureOutput_.c(),
                                                   mtState::nLevels_ - 1, false)) {
              mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[activeCamID], featureOutput_.c(),
                                                        mtState::nLevels_ - 1, false);
              mlpTemp1_.drawMultilevelPatch(filterState.patchDrawing_, cv::Point2i(
                      filterState.drawPB_ + (2 + 2 * activeCamID) * filterState.drawPS_,
                      filterState.drawPB_ + ID * filterState.drawPS_), 1, false);
            }
            if (f.mpStatistics_->status_[activeCamID] == TRACKED) {
              cv::rectangle(filterState.patchDrawing_,
                            cv::Point2i((2 + 2 * activeCamID) * filterState.drawPS_, ID * filterState.drawPS_),
                            cv::Point2i((3 + 2 * activeCamID) * filterState.drawPS_ - 1,
                                        (ID + 1) * filterState.drawPS_ - 1), cv::Scalar(0, 255, 0), 1, 8, 0);
            } else {
              cv::rectangle(filterState.patchDrawing_,
                            cv::Point2i((2 + 2 * activeCamID) * filterState.drawPS_, ID * filterState.drawPS_),
                            cv::Point2i((3 + 2 * activeCamID) * filterState.drawPS_ - 1,
                                        (ID + 1) * filterState.drawPS_ - 1), cv::Scalar(0, 0, 255), 1, 8, 0);
            }
          }
        }
        activeCamCounter++;
        if (activeCamCounter == mtState::nCam_ || !useCrossCameraMeasurements_) {
          activeCamCounter = 0;
          ID++;
        }
      }
    };

    /** \brief Final Post-Processing step for the image update.
     *
     *  Summary:
     *  1. For each feature in the state: Extract patches and compute Shi-Tomasi Score.
     *  2. Removal of bad features from the state.
     *  3. Get new features and add them to the state.
     *
     *  @param filterState      - Filter state.
     *  @param meas             - Update measurement.
     */
    void commonPostProcess(mtFilterState &filterState, const mtMeas &meas) {
      typename mtFilterState::mtState &state = filterState.state_;
      MXD &cov = filterState.cov_;

      int numAddedFeatures = 0;

      // Actualize camera extrinsics
      state.updateMultiCameraExtrinsics(mpMultiCamera_);

      int countTracked = 0;
      // For all features in the state.
      for (unsigned int i = 0; i < mtState::nMax_; i++) {
        if (filterState.fsm_.isValid_[i]) {
          FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_> &f = filterState.fsm_.features_[i];
          const int camID = f.mpCoordinates_->camID_;
          if (camID != meas.aux().activeModality_) {
            continue;
          }
          if (f.mpStatistics_->trackedInSomeFrame()) {
            countTracked++;
          }
          if (f.mpStatistics_->status_[camID] == TRACKED &&
              filterState.t_ - f.mpStatistics_->lastPatchUpdate_ > minTimeBetweenPatchUpdate_) {
            tempCoordinates_ = *f.mpCoordinates_;
            tempCoordinates_.set_warp_identity();
            if (mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[camID], tempCoordinates_, startLevel_, true)) {
              mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[camID], tempCoordinates_, startLevel_, true);
              mlpTemp1_.computeMultilevelShiTomasiScore(endLevel_, startLevel_);
              if (mlpTemp1_.s_ >= static_cast<float>(minAbsoluteSTScore_.at(camID)) &&
                  mlpTemp1_.s_ >= static_cast<float>(minRelativeSTScore_) * (f.mpMultilevelPatch_->s_)) {
                *f.mpMultilevelPatch_ = mlpTemp1_;
                f.mpCoordinates_->set_warp_identity();
                f.mpStatistics_->lastPatchUpdate_ = filterState.t_;
              }
            }
          }
          // Visualize Quatlity
          if (visualizePatches_) {
            for (int j = 0; j < mtState::nCam_; j++) {
              // Local Quality
              const double qLQ = f.mpStatistics_->getLocalQuality(j);
              cv::line(filterState.patchDrawing_,
                       cv::Point2i((2 + 2 * j) * filterState.drawPS_ + 1, (i + 1) * filterState.drawPS_ - 4),
                       cv::Point2i((2 + 2 * j) * filterState.drawPS_ + 1 + (filterState.drawPS_ - 3) * qLQ,
                                   (i + 1) * filterState.drawPS_ - 4), cv::Scalar(0, 255 * qLQ, 255 * (1 - qLQ)), 2, 8,
                       0);
            }
            const double qALQ = f.mpStatistics_->getAverageLocalQuality();
            cv::line(filterState.patchDrawing_, cv::Point2i(1, (i + 1) * filterState.drawPS_ - 10),
                     cv::Point2i(1 + (filterState.drawPS_ - 3) * qALQ, (i + 1) * filterState.drawPS_ - 10),
                     cv::Scalar(0, 255 * qALQ, 255 * (1 - qALQ)), 2, 8, 0);
            const double qLV = f.mpStatistics_->getJointLocalVisibility();
            cv::line(filterState.patchDrawing_, cv::Point2i(1, (i + 1) * filterState.drawPS_ - 7),
                     cv::Point2i(1 + (filterState.drawPS_ - 3) * qLV, (i + 1) * filterState.drawPS_ - 7),
                     cv::Scalar(0, 255 * qLV, 255 * (1 - qLV)), 2, 8, 0);
            const double qGQ = f.mpStatistics_->getGlobalQuality();
            cv::line(filterState.patchDrawing_, cv::Point2i(1, (i + 1) * filterState.drawPS_ - 4),
                     cv::Point2i(1 + (filterState.drawPS_ - 3) * qGQ, (i + 1) * filterState.drawPS_ - 4),
                     cv::Scalar(0, 255 * qGQ, 255 * (1 - qALQ)), 2, 8, 0);
          }
        }
      }

      // Removes features from cameras which are not active anymore. I.e. they stopped providing images. TODO: Asses the computational load from this and if is should be eclipsed in an if-statement to only run when necessary.
      for (int i = 0; i < mtState::nMax_; ++i) {
        if (filterState.fsm_.isValid_[i]) {
          FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_> &f = filterState.fsm_.features_[i];
          const int camID = f.mpCoordinates_->camID_;
          if (meas.aux().camActive_[camID]) {
            continue;
          }else{
            if (verbose_) std::cout << filterState.fsm_.features_[i].idx_ << ", ";
            filterState.fsm_.isValid_[i] = false;
            numRemovedFeaturesThisFrame += 1;
            filterState.resetFeatureCovariance(i, Eigen::Matrix3d::Identity());
            if (verbose_)std::cout << "Removed a feature due to dying camera.\n";
          }
        }
      }

      // Remove bad feature.
      float averageScore = filterState.fsm_.getAverageScore(); // TODO: make the following dependent on the ST-score
      if (verbose_) std::cout << "Removing features: ";
      for (unsigned int i = 0; i < mtState::nMax_; i++) {
        if (filterState.fsm_.isValid_[i]) {
          FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_> &f = filterState.fsm_.features_[i];
          const int camID = f.mpCoordinates_->camID_;
          if (camID != meas.aux().activeModality_) { // Custom: does not try to check features which was not updated
            continue;
          }
          if (!f.mpStatistics_->isGoodFeature(trackingUpperBound_, trackingLowerBound_)) {
            if (verbose_) std::cout << filterState.fsm_.features_[i].idx_ << ", ";
            filterState.fsm_.isValid_[i] = false;
            numRemovedFeaturesThisFrame += 1;
            filterState.resetFeatureCovariance(i, Eigen::Matrix3d::Identity());
          }
        }
      }
      if (verbose_) std::cout << " | ";
      // Check if enough free features, enforce removal
      int requiredFreeFeature = mtState::nMax_ * minTrackedAndFreeFeatures_ - countTracked;
      int possibleToRemove = 0; //  Sees how many untracked features there is in this modality
      for (int i = 0;i<mtState::nMax_; ++i){
        if (filterState.fsm_.isValid_[i] && filterState.fsm_.features_[i].mpCoordinates_->camID_ == meas.aux().activeModality_ && !filterState.fsm_.features_[i].mpStatistics_->trackedInSomeFrame()){
          ++possibleToRemove;
        }
      }
      double factor = removalFactor_;
      int featureIndex = 0;
      int fullIterations = 0;
      while (possibleToRemove>0 &&  (int) (mtState::nMax_) - (int) (filterState.fsm_.getValidCount()) < requiredFreeFeature) {
        if (filterState.fsm_.isValid_[featureIndex]) {
          FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_> &f = filterState.fsm_.features_[featureIndex];
          const int camID = f.mpCoordinates_->camID_;
          if (camID == meas.aux().activeModality_) {

            if (!f.mpStatistics_->trackedInSomeFrame() &&
                !f.mpStatistics_->isGoodFeature(trackingUpperBound_ * factor, trackingLowerBound_ * factor)) {
              if (verbose_) std::cout << filterState.fsm_.features_[featureIndex].idx_ << ", ";
              filterState.fsm_.isValid_[featureIndex] = false;
              numRemovedFeaturesThisFrame += 1;
              filterState.resetFeatureCovariance(featureIndex, Eigen::Matrix3d::Identity());
              possibleToRemove--;
            }
          }
        }
        featureIndex++;
        if (featureIndex == mtState::nMax_) {
          featureIndex = 0;
          factor = factor * removalFactor_;
          fullIterations++;
        }
      }
      if (verbose_) std::cout << std::endl;
      const int maxFramesBeforeRecheck = 20; // TODO: Make this an input parameter. Higher value gives less computational load
      // Get new features
      if (filterState.fsm_.getValidCount() < startDetectionTh_ * mtState::nMax_ || framesSinceLastDetectionAtempt_[meas.aux().activeModality_] > maxFramesBeforeRecheck) { // Custom have a max threshold per modality to detect changing conditions
        framesSinceLastDetectionAtempt_[meas.aux().activeModality_] = 0;
        // Compute the median depth parameters for each camera, using the state features.
        std::array<double, mtState::nCam_> medianDepthParameters;
        if (maxUncertaintyToDepthRatioForDepthInitialization_ > 0) {
          filterState.getMedianDepthParameters(initDepth_, &medianDepthParameters,
                                               maxUncertaintyToDepthRatioForDepthInitialization_);
        } else {
          medianDepthParameters.fill(initDepth_);
        }
        for (int camID = 0; camID < mtState::nCam_; camID++) {
          if (camID != meas.aux().activeModality_) {
            continue; // Don't find features in a modality which is not being updated
          }
          // Get Candidates
          const double t1 = (double) cv::getTickCount();
          candidates_.clear();
          //Histogram Equalization should only applied be if image is in 16-bit range
          bool applyHistogramEqualization = false;
          double imgMin, imgMax;
          cv::minMaxLoc(meas.aux().pyr_[camID].imgs_[startLevel_], &imgMin, &imgMax);
          if (imgMax > 255.0)
            applyHistogramEqualization = true;
          if (verbose_)
            std::cout << "Image Min/Max:" << imgMin << "/" << imgMax << "\t applyHistogramEqualization="
                      << applyHistogramEqualization << std::endl;

          //Detect FAST corners, equalize image if 16-bit before application of OpenCV FAST detector
          for (int l = endLevel_; l <= startLevel_; l++) {
            meas.aux().pyr_[camID].detectFastCorners(candidates_, l, fastDetectionThreshold_[camID], meas.aux().activeModality_,
                                                     std::numeric_limits<float>::max(), applyHistogramEqualization);
          }
          const double t2 = (double) cv::getTickCount();
          if (verbose_)
            std::cout << "== Detected " << candidates_.size() << " on levels " << endLevel_ << "-" << startLevel_
                      << " (" << (t2 - t1) / cv::getTickFrequency() * 1000 << " ms)" << std::endl;

          const double featureCountHistoryLength = 10.0; // TODO: Make this as an input parameter. Perhaps make it dependent on the frequency of the spesific camera.
          std::vector<MultilevelPatch<mtState::nLevels_,mtState::patchSize_>> multilevelPatches;
          goodFeatures.at(meas.aux().activeModality_).clear();
          float maxScore = filterState.fsm_.findCandidateScore(candidates_, meas.aux().pyr_[camID], endLevel_, startLevel_, minSTScoreForModalitySelection_[meas.aux().activeModality_], multilevelPatches, goodFeatures, numFoundFeatures_[camID], filterState.t_, additionalScoreFromOtherModality_.at(camID), maxDistanceForApproxamentlyLocalizingFeatures_);
          auto start = std::chrono::steady_clock::now();
          double timeSinceLastCall = 0.0;
          if (featureCountHistory_[camID] < 0){
            previousUpdateOfFeatureCountHistory_[camID] = filterState.t_;
            timeSinceLastCall = 0.0;
          }else{
            timeSinceLastCall = filterState.t_ - previousUpdateOfFeatureCountHistory_[camID];
            previousUpdateOfFeatureCountHistory_[camID] = filterState.t_;
          }
          const int maxFromModality = maxFeaturesFromThisModality(filterState, meas, camID,
                                                                  numFoundFeatures_, 1.0, 0.0, featureCountHistory_,
                                                                  featureCountHistoryLength, timeSinceLastCall);
          const int maxAddedFeatures = std::min((mtState::nMax_ - filterState.fsm_.getValidCount()) / (mtState::nCam_ - camID), maxFromModality);

          std::unordered_set<unsigned int> newSet;
          if(maxScore > minAbsoluteSTScore_[camID]){
            newSet = filterState.fsm_.bucketCandidates(candidates_, camID, maxAddedFeatures, nDetectionBuckets_, scoreDetectionExponent_,
                                      penaltyDistance_, zeroDistancePenalty_,false, minAbsoluteSTScore_[camID], maxScore, multilevelPatches);
          }

          numAddedFeatures = newSet.size();
          const double t3 = (double) cv::getTickCount();
          if (verbose_)
            std::cout << "== Got " << filterState.fsm_.getValidCount() << " after adding " << newSet.size()
                      << " features in camera " << camID << " (" << (t3 - t2) / cv::getTickFrequency() * 1000 << " ms)"
                      << std::endl;
          for (auto it = newSet.begin(); it != newSet.end(); ++it) {
            FeatureManager<mtState::nLevels_, mtState::patchSize_, mtState::nCam_> &f = filterState.fsm_.features_[*it];
            f.mpStatistics_->resetStatistics(filterState.t_);
            f.mpStatistics_->status_[camID] = TRACKED;
            f.mpStatistics_->lastPatchUpdate_ = filterState.t_;
            f.mpDistance_->p_ = medianDepthParameters[camID];
            const float initRelDepthCovTemp_ = initCovFeature_(0, 0);
            initCovFeature_(0, 0) = initRelDepthCovTemp_ *
                                    pow(f.mpDistance_->getParameterDerivative() * f.mpDistance_->getDistance(), 2);
            filterState.resetFeatureCovariance(*it, initCovFeature_);
            initCovFeature_(0, 0) = initRelDepthCovTemp_;
            if (doFrameVisualisation_) {
              f.mpCoordinates_->drawPoint(filterState.img_[camID], cv::Scalar(255, 0, 0));
              f.mpCoordinates_->drawText(filterState.img_[camID], std::to_string(f.idx_), cv::Scalar(255, 0, 0));
            }

            if (mtState::nCam_ > 1 && doStereoInitialization_) {
              const int otherCam = (camID + 1) % mtState::nCam_;
              transformFeatureOutputCT_.setFeatureID(*it);
              transformFeatureOutputCT_.setOutputCameraID(otherCam);
              transformFeatureOutputCT_.transformState(filterState.state_, featureOutput_);
              if (alignment_.align2DAdaptive(alignedCoordinates_, meas.aux().pyr_[otherCam], *f.mpMultilevelPatch_,
                                             featureOutput_.c(), startLevel_, endLevel_,
                                             alignConvergencePixelRange_, alignCoverageRatio_, alignMaxUniSample_)) {
                bool valid = mlpTemp1_.isMultilevelPatchInFrame(meas.aux().pyr_[otherCam], alignedCoordinates_,
                                                                startLevel_, false);
                if (valid && patchRejectionTh_[camID] >= 0) {
                  mlpTemp1_.extractMultilevelPatchFromImage(meas.aux().pyr_[otherCam], alignedCoordinates_, startLevel_,
                                                            false);
                  const float avgError = mlpTemp1_.computeAverageDifference(*f.mpMultilevelPatch_, endLevel_,
                                                                            startLevel_);
                  if (avgError > patchRejectionTh_[camID]) {
                    valid = false;
                  }
                }
                if (valid == true) {
                  if (doFrameVisualisation_) {
                    alignedCoordinates_.drawPoint(filterState.img_[otherCam], cv::Scalar(150, 0, 0));
                    alignedCoordinates_.drawText(filterState.img_[otherCam], std::to_string(f.idx_),
                                                 cv::Scalar(150, 0, 0));
                  }
                  if (f.mpCoordinates_->getDepthFromTriangulation(alignedCoordinates_, state.qCM(otherCam).rotate(
                          V3D(state.MrMC(camID) - state.MrMC(otherCam))),
                                                                  state.qCM(otherCam) * state.qCM(camID).inverted(),
                                                                  *f.mpDistance_, 0.01)) {
                    filterState.resetFeatureCovariance(*it, initCovFeature_); // TODO: improve
                  }
                } else {
                  if (doFrameVisualisation_) {
                    alignedCoordinates_.drawPoint(filterState.img_[otherCam], cv::Scalar(0, 0, 150));
                    alignedCoordinates_.drawText(filterState.img_[otherCam], std::to_string(f.idx_),
                                                 cv::Scalar(0, 0, 150));
                  }
                }
              } else {
                if (doFrameVisualisation_) {
                  alignedCoordinates_.drawPoint(filterState.img_[otherCam], cv::Scalar(0, 150, 0));
                  alignedCoordinates_.drawText(filterState.img_[otherCam], std::to_string(f.idx_),
                                               cv::Scalar(0, 150, 0));
                }
              }
            }
          }
        }
      }else{
        // We did not consider this frames since there was enough features already
        framesSinceLastDetectionAtempt_[meas.aux().activeModality_]++;
      }
      for (unsigned int i = 0; i < mtState::nMax_; i++) {
        if (filterState.fsm_.isValid_[i]) {
          filterState.fsm_.features_[i].log_previous_ = *filterState.fsm_.features_[i].mpCoordinates_;
        }
      }
      if (doFrameVisualisation_) {
        for (int i = 0; i < mtState::nCam_; i++) {
          drawVirtualHorizon(filterState, i);
        }
      }

      if (verbose_) {
        for (int i = 0; i < mtState::nCam_; i++) {
          std::cout << "Camera extrinsics: " << i << std::endl;
          std::cout << "  " << filterState.state_.qCM(i) << std::endl;
          std::cout << "  " << filterState.state_.MrMC(i).transpose() << std::endl;
        }
      }

      filterState.prevPyr_[meas.aux().activeModality_] = meas.aux().pyr_[meas.aux().activeModality_];

      // Zero Velocity updates if appropriate
      if (isZeroVelocityUpdateEnabled_
          && doVisualMotionDetection_ &&
          filterState.state_.aux().timeSinceLastImageMotion_ > minTimeForZeroVelocityUpdate_
          && filterState.state_.aux().timeSinceLastInertialMotion_ > minTimeForZeroVelocityUpdate_) {
        cv::putText(filterState.img_[0], "Performing Zero Velocity Updates!", cv::Point2f(150, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255));
        zeroVelocityUpdate_.performUpdateEKF(filterState, ZeroVelocityUpdateMeas<mtState>());
      }

      // Stores the relevant features statistics
      if (collectFeatureStatistics){
        int activeFeaturesInThisModality = 0;
        for (int i = 0;i<mtState::nMax_; ++i){
          if (filterState.fsm_.isValid_[i] && filterState.fsm_.features_[i].mpCoordinates_->camID_ == meas.aux().activeModality_ && filterState.fsm_.features_[i].mpStatistics_->trackedInSomeFrame()){
            ++activeFeaturesInThisModality;
          }
        }
        double averageFeatureLifetime = 0;
        int totalFeatures = 0;
        for (int i = 0;i<mtState::nMax_; ++i){
          if (filterState.fsm_.isValid_[i] && filterState.fsm_.features_[i].mpCoordinates_->camID_ == meas.aux().activeModality_ && filterState.fsm_.features_[i].mpStatistics_->trackedInSomeFrame()){
            const FeatureStatistics<mtState::nCam_> featureStatistics = *filterState.fsm_.features_[i].mpStatistics_;
            averageFeatureLifetime +=   featureStatistics.currentTime_ - featureStatistics.initTime_;
            totalFeatures++;
          }
        }
        averageFeatureLifetime = averageFeatureLifetime / (double) totalFeatures;
        featureStatisticsFileOut_ << meas.aux().imgTime_ << ", " << meas.aux().activeModality_ << ", " << activeFeaturesInThisModality << ", " << numAddedFeatures << ", " << numRemovedFeaturesThisFrame << ", " << averageFeatureLifetime << "\n";
        featureStatisticsFileOut_.flush();
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /** \brief Draws a virtual horizon into the current image of the camera with ID camID
     *
     *  @param filterState - Filter state.
     *  @param camID       - ID of the camera, in which image the horizon should be drawn.
     */
    void drawVirtualHorizon(mtFilterState &filterState, const int camID = 0) {
      typename mtFilterState::mtState &state = filterState.state_;
      cv::rectangle(filterState.img_[camID], cv::Point2f(0, 0), cv::Point2f(82, 92), cv::Scalar(50, 50, 50), -1, 8, 0);
      cv::rectangle(filterState.img_[camID], cv::Point2f(0, 0), cv::Point2f(80, 90), cv::Scalar(100, 100, 100), -1, 8,
                    0);
      cv::putText(filterState.img_[camID], std::to_string(filterState.imageCounter_), cv::Point2f(5, 85),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0));
      cv::Point2f rollCenter = cv::Point2f(40, 40);
      cv::Scalar rollColor1(50, 50, 50);
      cv::Scalar rollColor2(200, 200, 200);
      cv::Scalar rollColor3(120, 120, 120);
      cv::circle(filterState.img_[camID], rollCenter, 32, rollColor1, -1, 8, 0);
      cv::circle(filterState.img_[camID], rollCenter, 30, rollColor2, -1, 8, 0);
      Eigen::Vector3d Vg = (state.qCM(camID) * state.qWM().inverted()).rotate(Eigen::Vector3d(0, 0, -1));
      double roll = atan2(Vg(1), Vg(0)) - 0.5 * M_PI;
      double pitch = acos(Vg.dot(Eigen::Vector3d(0, 0, 1))) - 0.5 * M_PI;
      double pixelFor10Pitch = 5.0;
      double pitchOffsetAngle = -asin(pitch / M_PI * 180.0 / 10.0 * pixelFor10Pitch / 30.0);
      cv::Point2f rollVector1 = 30 * cv::Point2f(cos(roll), sin(roll));
      cv::Point2f rollVector2 = cv::Point2f(25, 0);
      cv::Point2f rollVector3 = cv::Point2f(10, 0);
      std::vector<cv::Point> pts;
      cv::ellipse2Poly(rollCenter, cv::Size(30, 30), 0, (roll - pitchOffsetAngle) / M_PI * 180,
                       (roll + pitchOffsetAngle) / M_PI * 180 + 180, 1, pts);
      cv::Point *points;
      points = &pts[0];
      int nbtab = pts.size();
      cv::fillPoly(filterState.img_[camID], (const cv::Point **) &points, &nbtab, 1, rollColor3);
      cv::line(filterState.img_[camID], rollCenter + rollVector2, rollCenter + rollVector3, rollColor1, 2);
      cv::line(filterState.img_[camID], rollCenter - rollVector2, rollCenter - rollVector3, rollColor1, 2);
      cv::ellipse(filterState.img_[camID], rollCenter, cv::Size(10, 10), 0, 0, 180, rollColor1, 2, 8, 0);
      cv::circle(filterState.img_[camID], rollCenter, 2, rollColor1, -1, 8, 0);
    }
  };

}


#endif /* ROVIO_IMGUPDATE_HPP_ */
