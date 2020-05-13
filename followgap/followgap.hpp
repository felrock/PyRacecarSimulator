#pragma once
#include <vector>
#include <cmath>

class FollowGap
{
    private:
        int window_size;
        float max_distance;
        float max_angle;
        float angle_inc;

    public:

        FollowGap(int ws, float md, float ma, float angle_inc) : window_size(ws),
                    max_distance(md), max_angle(ma), angle_inc(angle_inc) {}

        inline void preprocessLidar(std::vector<float> &lidar)
        {

            for(int i=0; i < lidar.size()-10; ++i)
            {
                if(lidar[i] > max_distance)
                {
                    lidar[i] = max_distance;
                }
            }
        }

        inline std::pair<int, int> findMaxGap(std::vector<float> lidar)
        {
            int current_start = 0;
            int current_size = 0;
            int max_start = 0;
            int max_size = 0;

            int c_index = 0;

            while(c_index < lidar.size())
            {
                current_start = c_index;
                current_size = 0;
                while(c_index < lidar.size() &&  lidar[c_index] > 1.75)
                {
                    current_size++;
                    c_index++;
                }
                if(current_size > max_size)
                {
                    max_start = current_start;
                    max_size = current_size;
                    current_size = 0;
                }
                c_index++;
            }
            if(current_size > max_size)
            {
                return std::pair<int, int>(current_start,
                                        current_start + current_size + 1);
            }
            else
            {
                return std::pair<int, int>(max_start, max_start + max_size + 1);
            }
        }

        inline void safetyBubble(std::vector<float> &lidar, int center,
                                            int radius)
        {
            lidar[center] = 0.0;
            int c_index = center;

            for(int i=-radius; i < radius; ++i)
            {
                if(c_index + i > 0 && c_index + i < lidar.size()-1)
                {
                    lidar[c_index+i] = 0.0;
                }
            }
        }

        inline float getSteerAng(float d, int size, int best_point)
        {
            float angle;
            if(best_point > size/2)
            {
                angle = - angle_inc *((size/2.0) - best_point);
            }
            else
            {
                angle = angle_inc *(best_point - (size/2.0));
            }

            angle = 2 * (angle / d);

            return std::min(std::max(angle, -max_angle), max_angle);
        }

        inline int findBestPoint(int start, int end)
        {
            return (start + end)/2;
        }

        float eval(float* lidar, int size)
        {
            std::vector<float> v;
            for(int i=0; i < size; ++i)
                v.push_back(lidar[i]);

            preprocessLidar(v);

            int min_point = 0;
            for(int i=0; i < size; ++i)
            {
                if(v[i] != 0 && v[i] < v[min_point])
                {
                    min_point = i;
                }
            }
            safetyBubble(v, min_point, 5);

            std::pair<int, int> gap = findMaxGap(v);

            int best_point = findBestPoint(gap.first, gap.second);

            float angle = getSteerAng(lidar[best_point], v.size(), best_point);

            return angle;
        }
};
