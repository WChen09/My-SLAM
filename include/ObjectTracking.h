#pragma once
#include <opencv2/opencv.hpp>
#include <assert.h>

#include "HungarianAlg.h"
#include <vector>
#include "Thirdparty/darknet/src/object.h"
#include <list>
/**
 * @brief nms
 * Non maximum suppression
 * @param srcRects
 * @param resRects
 * @param thresh
 * @param neighbors
 */
inline void nms(
        const std::vector<cv::Rect>& srcRects,
        std::vector<cv::Rect>& resRects,
        float thresh,
        int neighbors = 0
        )
{
    resRects.clear();

    const size_t size = srcRects.size();
    if (!size)
    {
        return;
    }

    // Sort the bounding boxes by the bottom - right y - coordinate of the bounding box
    std::multimap<int, size_t> idxs;
    for (size_t i = 0; i < size; ++i)
    {
        idxs.insert(std::pair<int, size_t>(srcRects[i].br().y, i));
    }

    // keep looping while some indexes still remain in the indexes list
    while (idxs.size() > 0)
    {
        // grab the last rectangle
        auto lastElem = --std::end(idxs);
        const cv::Rect& rect1 = srcRects[lastElem->second];

        int neigborsCount = 0;

        idxs.erase(lastElem);

        for (auto pos = std::begin(idxs); pos != std::end(idxs); )
        {
            // grab the current rectangle
            const cv::Rect& rect2 = srcRects[pos->second];

            float intArea = (rect1 & rect2).area();
            float unionArea = rect1.area() + rect2.area() - intArea;
            float overlap = intArea / unionArea;

            // if there is sufficient overlap, suppress the current bounding box
            if (overlap > thresh)
            {
                pos = idxs.erase(pos);
                ++neigborsCount;
            }
            else
            {
                ++pos;
            }
        }
        if (neigborsCount >= neighbors)
        {
            resRects.push_back(rect1);
        }
    }
}

/**
 * @brief nms2
 * Non maximum suppression with detection scores
 * @param srcRects
 * @param scores
 * @param resRects
 * @param thresh
 * @param neighbors
 */
inline void nms2(
        const std::vector<cv::Rect>& srcRects,
        const std::vector<float>& scores,
        std::vector<cv::Rect>& resRects,
        float thresh,
        int neighbors = 0,
        float minScoresSum = 0.f
        )
{
    resRects.clear();

    const size_t size = srcRects.size();
    if (!size)
    {
        return;
    }

    assert(srcRects.size() == scores.size());

    // Sort the bounding boxes by the detection score
    std::multimap<float, size_t> idxs;
    for (size_t i = 0; i < size; ++i)
    {
        idxs.insert(std::pair<float, size_t>(scores[i], i));
    }

    // keep looping while some indexes still remain in the indexes list
    while (idxs.size() > 0)
    {
        // grab the last rectangle
        auto lastElem = --std::end(idxs);
        const cv::Rect& rect1 = srcRects[lastElem->second];

        int neigborsCount = 0;
        float scoresSum = lastElem->first;

        idxs.erase(lastElem);

        for (auto pos = std::begin(idxs); pos != std::end(idxs); )
        {
            // grab the current rectangle
            const cv::Rect& rect2 = srcRects[pos->second];

            float intArea = (rect1 & rect2).area();
            float unionArea = rect1.area() + rect2.area() - intArea;
            float overlap = intArea / unionArea;

            // if there is sufficient overlap, suppress the current bounding box
            if (overlap > thresh)
            {
                scoresSum += pos->first;
                pos = idxs.erase(pos);
                ++neigborsCount;
            }
            else
            {
                ++pos;
            }
        }
        if (neigborsCount >= neighbors &&
                scoresSum >= minScoresSum)
        {
            resRects.push_back(rect1);
        }
    }
}
inline double CalcDistJaccard(cv::Rect &current, cv::Rect &former)
{

    float intArea = (current & former).area();
    float unionArea = current.area() + former.area() - intArea;

    return 1 - intArea / unionArea;
}

inline void HungarianAssignment(std::vector<int>& _assignment,
                                std::vector<DetectedObject>& LastObjects,
                                std::vector<DetectedObject>& CurrentObjects,
                                const float maxDisTh)
{
    size_t N = LastObjects.size();
    size_t M = CurrentObjects.size();

    _assignment.resize(N, -1);
    std::vector<float> Distance(N*M);
    int k = 0;
    for(size_t i = 0; i < _assignment.size(); i++)
    {

        DetectedObject lastBox = LastObjects.at(i);
        for(size_t j = 0; j < M; j++)
        {
            float dist =  CalcDistJaccard(CurrentObjects.at(j).bounding_box, lastBox.bounding_box);
            Distance.at(i + j * N) = dist;
        }
    }
    AssignmentProblemSolver APS;
    APS.Solve(Distance, N, M, _assignment, AssignmentProblemSolver::optimal);

    for (size_t i = 0; i < _assignment.size(); i++)
    {
        if (_assignment.at(i) != -1)
        {
            if (Distance.at(i + _assignment.at(i) * N) > maxDisTh)
            {
                _assignment.at(i) = -1;
            }
        }
    }

}

