#include "voxel_map_util.hpp"


// check is plane , calc plane parameters including plane covariance
//~ 这个函数直接“计算”新的平面参数（平面中心、法向量、协方差），并同时判断是否是一个平面。
void OctoTree::init_plane(const std::vector<pointWithCov> &points, Plane *plane) {
    plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance = Eigen::Matrix3d::Zero();
    plane->center = Eigen::Vector3d::Zero();
    plane->normal = Eigen::Vector3d::Zero();
    plane->points_size = points.size();
    plane->radius = 0;
    for (auto pv : points) {
        plane->covariance += pv.point * pv.point.transpose();
        plane->center += pv.point;
    }
    plane->center = plane->center / plane->points_size;
    plane->covariance = plane->covariance / plane->points_size - plane->center * plane->center.transpose();   //~ 计算加入的这些points的协方差
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
   ::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3f::Index evalsMin, evalsMax;              //~ 获得最大、最小、和中间特征值的位置（对应eigen-value的行）
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;                 //~ 计算中大小特征值的位置
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);   //~ 获得对应的特征向量
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
    // plane covariance calculation
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size;
    if (evalsReal(evalsMin) < planer_threshold_) {
        std::vector<int> index(points.size());
        std::vector<Eigen::Matrix<double, 6, 6>> temp_matrix(points.size());
        for (int i = 0; i < points.size(); i++) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F;
            for (int m = 0; m < 3; m++) {
                if (m != (int)evalsMin) {
                    Eigen::Matrix<double, 1, 3> F_m =
                            (points[i].point - plane->center).transpose() /
                            ((plane->points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
                            (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
                    F.row(m) = F_m;
                } else {
                    Eigen::Matrix<double, 1, 3> F_m;
                    F_m << 0, 0, 0;
                    F.row(m) = F_m;
                }
            }
            J.block<3, 3>(0, 0) = evecs.real() * F;
            J.block<3, 3>(3, 0) = J_Q;
            plane->plane_cov += J * points[i].cov * J.transpose();
        }

        plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
        // plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
        plane->x_normal = crossProduct(plane->y_normal, plane->normal);

        plane->min_eigen_value = evalsReal(evalsMin);
        plane->mid_eigen_value = evalsReal(evalsMid);
        plane->max_eigen_value = evalsReal(evalsMax);
        plane->radius = sqrt(evalsReal(evalsMax));
        plane->d = -(plane->normal(0) * plane->center(0) +
                plane->normal(1) * plane->center(1) +
                plane->normal(2) * plane->center(2));
        plane->is_plane = true;
        if (plane->last_update_points_size == 0) {
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        } else if (plane->points_size - plane->last_update_points_size > 100) {
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        }

        if (!plane->is_init) {
            plane->id = plane_id;
            plane_id++;
            plane->is_init = true;
        }

    } else {
        if (!plane->is_init) {
            plane->id = plane_id;
            plane_id++;
            plane->is_init = true;
        }
        if (plane->last_update_points_size == 0) {
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        } else if (plane->points_size - plane->last_update_points_size > 100) {
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        }
        plane->is_plane = false;
        plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
        // plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
        plane->x_normal = crossProduct(plane->y_normal, plane->normal);

        plane->min_eigen_value = evalsReal(evalsMin);
        plane->mid_eigen_value = evalsReal(evalsMid);
        plane->max_eigen_value = evalsReal(evalsMax);
        plane->radius = sqrt(evalsReal(evalsMax));
        plane->d = -(plane->normal(0) * plane->center(0) +
                plane->normal(1) * plane->center(1) +
                plane->normal(2) * plane->center(2));
    }
}

// only updaye plane normal, center and radius with new points
//~ 这个函数不“计算”，而是“更新”平面参数（平面中心、法向量）。但是，代码中并没有判断，是否为同一个平面？ISSUE:
void OctoTree::update_plane(const std::vector<pointWithCov> &points, Plane *plane) {
    Eigen::Matrix3d old_covariance = plane->covariance;
    Eigen::Vector3d old_center = plane->center;
    Eigen::Matrix3d sum_ppt =
            (plane->covariance + plane->center * plane->center.transpose()) *
            plane->points_size;
    Eigen::Vector3d sum_p = plane->center * plane->points_size;
    for (size_t i = 0; i < points.size(); i++) {
        Eigen::Vector3d pv = points[i].point;
        sum_ppt += pv * pv.transpose();
        sum_p += pv;
    }
    plane->points_size = plane->points_size + points.size();
    plane->center = sum_p / plane->points_size;
    plane->covariance = sum_ppt / plane->points_size - plane->center * plane->center.transpose();
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3d::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
    if (evalsReal(evalsMin) < planer_threshold_) {
        plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
        // plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
        plane->x_normal = crossProduct(plane->y_normal, plane->normal);

        plane->min_eigen_value = evalsReal(evalsMin);
        plane->mid_eigen_value = evalsReal(evalsMid);
        plane->max_eigen_value = evalsReal(evalsMax);
        plane->radius = sqrt(evalsReal(evalsMax));
        plane->d = -(plane->normal(0) * plane->center(0) +
                plane->normal(1) * plane->center(1) +
                plane->normal(2) * plane->center(2));

        plane->is_plane = true;
        plane->is_update = true;
    } else {
        //~ 这段代码和上面if的几乎一样？只有is_plane这一行是false
        plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
        // plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
        plane->x_normal = crossProduct(plane->y_normal, plane->normal);

        plane->min_eigen_value = evalsReal(evalsMin);
        plane->mid_eigen_value = evalsReal(evalsMid);
        plane->max_eigen_value = evalsReal(evalsMax);
        plane->radius = sqrt(evalsReal(evalsMax));
        plane->d = -(plane->normal(0) * plane->center(0) +
                plane->normal(1) * plane->center(1) +
                plane->normal(2) * plane->center(2));
        plane->is_plane = false;
        plane->is_update = true;
    }
}

void OctoTree::init_octo_tree() {
    if (temp_points_.size() > max_plane_update_threshold_) {    //~ 每一个Voxel都是一个Octo-tree。检查点数够，再执行
        init_plane(temp_points_, plane_ptr_);
        if (plane_ptr_->is_plane == true) {
            octo_state_ = 0;            //~ 如果符合平面，就标记为tree的叶子节点，不再进行拆分平面。
            if (temp_points_.size() > max_cov_points_size_) {
                update_cov_enable_ = false;
            }
            if (temp_points_.size() > max_points_size_) {
                update_enable_ = false;
            }
        } else {                      //~ 否则，标记为1，再拆分。
            octo_state_ = 1;
            cut_octo_tree();            //~ 继续拆分，直到符合平面。
        }
        init_octo_ = true;            //~ 只有被初始化以后的tree，在能够在后续update时使用
        new_points_num_ = 0;
        //      temp_points_.clear();
    }
}

void OctoTree::cut_octo_tree() {
    if (layer_ >= max_layer_) {     //~ 树太深了，不拆了
        octo_state_ = 0;
        return;
    }
    for (size_t i = 0; i < temp_points_.size(); i++) {  //~ 首先判断这个点位于要拆分后的八叉树中的哪一块，然后各自创建
        int xyz[3] = {0, 0, 0};
        if (temp_points_[i].point[0] > voxel_center_[0]) {
            xyz[0] = 1;
        }
        if (temp_points_[i].point[1] > voxel_center_[1]) {
            xyz[1] = 1;
        }
        if (temp_points_[i].point[2] > voxel_center_[2]) {
            xyz[2] = 1;
        }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] == nullptr) {
            leaves_[leafnum] = new OctoTree(max_layer_, layer_ + 1, layer_point_size_, max_points_size_, max_cov_points_size_, planer_threshold_);
            leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
        }
        leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
        leaves_[leafnum]->new_points_num_++;
    }
    for (uint i = 0; i < 8; i++) {                      //~ 再递归检查每个八叉树，判断是否创建平面或拆分。
        if (leaves_[i] != nullptr) {
            if (leaves_[i]->temp_points_.size() > leaves_[i]->max_plane_update_threshold_) {
                init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
                if (leaves_[i]->plane_ptr_->is_plane) {
                    leaves_[i]->octo_state_ = 0;
                } 
                else {
                    leaves_[i]->octo_state_ = 1;
                    leaves_[i]->cut_octo_tree();
                }
                leaves_[i]->init_octo_ = true;
                leaves_[i]->new_points_num_ = 0;
            }
        }
    }
}

/*
初始状态判断：
1、!init_octo_：如果八叉树还没有初始化，那么每当有新的点进来时，先将其暂存到 temp_points_ 中。如果暂存点数超过阈值 max_plane_update_threshold_，则初始化八叉树。
2、八叉树已经初始化：
八叉树初始化后，首先根据 plane_ptr_->is_plane 判断当前节点是否为平面节点：
    平面节点的处理：
        如果 update_enable_ 为真，说明平面可以被更新。
        根据 update_cov_enable_ 判断是否更新协方差。
        新点数超过 update_size_threshold_ 时，初始化或更新平面。
        当所有点的数量超过 max_cov_points_size_ 或 max_points_size_ 时，禁用协方差更新或禁用更新功能。
    非平面节点的处理：
        如果当前层 layer_ 小于最大层数 max_layer_，则计算点所在的子叶节点，如果子节点存在则递归更新该子节点，如果不存在则创建一个新的子节点并更新。
        如果当前层数等于 max_layer_，则和平面节点类似，执行点的更新和条件判断。
*/
void OctoTree::UpdateOctoTree(const pointWithCov &pv) {
    // 1. 还未初始化，暂存。
    if (!init_octo_) {            //~ init_octo_：在`init_octo_tree`中有足够多的点以后，判定为true；或
        new_points_num_++;
        all_points_num_++;
        temp_points_.push_back(pv);
        if (temp_points_.size() > max_plane_update_threshold_) {
            init_octo_tree();
        }
    }
    // 2. 已经初始化，则继续判断
    else {
        // 2.1 是平面节点，则判断是否可以进行更新 (update_enable_)。
        if (plane_ptr_->is_plane) {
            // 2.1.1 update_enable_ 决定了是否进行更新。默认可以，不能更新的2个情况：
                // 1）当所有点的数量 all_points_num_ 达到或超过了 max_points_size_。
                // 2）非平面节点拆分达到了最大层数。
            if (update_enable_) {
                new_points_num_++;
                all_points_num_++;
                // 2.1.2.1 然后再判断是否更新协方差。和上方类似，超过了点数 max_cov_points_size_ 或到最大层数。目前 max_points_size_ 和 max_cov_points_size_ 设定一致，所以更新依据好像是一样的？ TODO: 
                // 如果cov是可以更新的，会将新的点存在temp_points_中用于后续更新。否则，存在new_points中。
                if (update_cov_enable_) {
                    temp_points_.push_back(pv);
                }  
                else {
                    new_points_.push_back(pv);    
                    //~ 注意，这个值好像没有用。此时状态是，“是个平面，但不更新协方差”，所以new_points_后面没有用到，就清空了。
                    //~ 莫非是？是个平面，但在某次判定后，判定为不是平面以后，后面再次参与计算了？此时有可能没有被清空。
                }

                // 2.1.2.2 当temp_points_足够多以后，会init平面。在init_plane中。ISSUE: 和后面update_plane有什么区别？
                if (new_points_num_ > update_size_threshold_) {     //~ 5
                    if (update_cov_enable_) {
                        init_plane(temp_points_, plane_ptr_);
                    }
                    //TODO: ISSUE: 为什么这里 Update_cov_enable_是false时，不再更新平面？ update_plane?
                    new_points_num_ = 0;
                }

                // 2.1.2.3 如果用于更新协方差的点数足够了，则清空所有temp_points_;
                if (all_points_num_ >= max_cov_points_size_) {
                    update_cov_enable_ = false;
                    std::vector<pointWithCov>().swap(temp_points_);
                }
                // 2.1.2.4 如果总点数足够多了，则清空所有new_points_。目前参数设定上，这两个if会同时执行。 ISSUE: 搞清楚temp_points_和new_points_的关系。
                if (all_points_num_ >= max_points_size_) {
                    update_enable_ = false;
                    plane_ptr_->update_enable = false;      //~ 这个变量没有用到。和Voxel的update_enable_动作一致
                    std::vector<pointWithCov>().swap(new_points_);
                }
            } 
            // 2.1.2 不能够更新, update_enable_，则直接结束。 
            // 什么时候“是平面”但“不能够更新”呢？点数足够多了，所以这时候这个OctoTree就啥也不需要做了。
            else {
                return;
            }
        }

        // 2.2 非平面的节点。
        else {
            // 2.2.1 未达到最大深度，则继续往下创建新的节点。
            if (layer_ < max_layer_) {
                if (temp_points_.size() != 0) {
                    std::vector<pointWithCov>().swap(temp_points_);
                }
                if (new_points_.size() != 0) {
                    std::vector<pointWithCov>().swap(new_points_);
                }
                //~ ? 这段在做什么，判断leaves_
                int xyz[3] = {0, 0, 0};
                if (pv.point[0] > voxel_center_[0]) {
                    xyz[0] = 1;
                }
                if (pv.point[1] > voxel_center_[1]) {
                    xyz[1] = 1;
                }
                if (pv.point[2] > voxel_center_[2]) {
                    xyz[2] = 1;
                }
                int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                if (leaves_[leafnum] != nullptr) {
                    leaves_[leafnum]->UpdateOctoTree(pv);
                } 
                else {
                    leaves_[leafnum] = new OctoTree(
                            max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                            max_cov_points_size_, planer_threshold_);
                    leaves_[leafnum]->layer_point_size_ = layer_point_size_;
                    leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                    leaves_[leafnum]->quater_length_ = quater_length_ / 2;
                    leaves_[leafnum]->UpdateOctoTree(pv);
                }
            } 
            // 2.2.2 已经达到了最大深度。这个节点不会再更新了。如果还处于可更新状态，则改成不可更新。
            else {
                // 2.2.2.1 状态上是否可以继续更新？若不是，则跳过；若是，则改成不可更新。
                if (update_enable_) {
                    new_points_num_++;
                    all_points_num_++;
                    if (update_cov_enable_) {
                        temp_points_.push_back(pv);
                    } else {
                        new_points_.push_back(pv);
                    }
                    if (new_points_num_ > update_size_threshold_) {
                        if (update_cov_enable_) {
                            init_plane(temp_points_, plane_ptr_);
                        } else {
                            update_plane(new_points_, plane_ptr_);
                            new_points_.clear();
                        }
                        new_points_num_ = 0;
                    }
                    if (all_points_num_ >= max_cov_points_size_) {
                        update_cov_enable_ = false;
                        std::vector<pointWithCov>().swap(temp_points_);
                    }
                    if (all_points_num_ >= max_points_size_) {
                        update_enable_ = false;
                        plane_ptr_->update_enable = false;      //~ 这个变量没有用到。和Voxel的update_enable_动作一致
                        std::vector<pointWithCov>().swap(new_points_);
                    }
                }
            }
        }
    }
}






//////////////////////////////////
//~ Set color based on v.
void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b) {
    r = 255;
    g = 255;
    b = 255;

    if (v < vmin) {
        v = vmin;
    }

    if (v > vmax) {
        v = vmax;
    }

    double dr, dg, db;

    if (v < 0.1242) {
        db = 0.504 + ((1. - 0.504) / 0.1242) * v;
        dg = dr = 0.;
    } else if (v < 0.3747) {
        db = 1.;
        dr = 0.;
        dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
    } else if (v < 0.6253) {
        db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
        dg = 1.;
        dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
    } else if (v < 0.8758) {
        db = 0.;
        dr = 1.;
        dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
    } else {
        db = 0.;
        dg = 0.;
        dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
    }

    r = (uint8_t)(255 * dr);
    g = (uint8_t)(255 * dg);
    b = (uint8_t)(255 * db);
}

void buildVoxelMap(const std::vector<pointWithCov> &input_points,
                   const float voxel_size, const int max_layer,
                   const std::vector<int> &layer_point_size,
                   const int max_points_size, const int max_cov_points_size,
                   const float planer_threshold,
                   std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map) {
    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
        const pointWithCov p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = feat_map.find(position);      //~ feat_map 外部传入的就是VoxelMap，Hash映射的，VOXEL_LOC -> OctoTree
        if (iter != feat_map.end()) {
            feat_map[position]->temp_points_.push_back(p_v);
            feat_map[position]->new_points_num_++;
        } else {
            OctoTree *octo_tree = new OctoTree(max_layer, 0, layer_point_size, max_points_size, max_cov_points_size, planer_threshold);
            feat_map[position] = octo_tree;
            feat_map[position]->quater_length_ = voxel_size / 4;
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->temp_points_.push_back(p_v);
            feat_map[position]->new_points_num_++;
            feat_map[position]->layer_point_size_ = layer_point_size;
        }
    }
    for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
        iter->second->init_octo_tree();       //~ 对每一个voxel创建一个octo_tree
    }
}

void updateVoxelMap(const std::vector<pointWithCov> &input_points,
                                        const float voxel_size, const int max_layer,
                                        const std::vector<int> &layer_point_size,
                                        const int max_points_size, const int max_cov_points_size,
                                        const float planer_threshold,
                                        std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map) {
    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
        const pointWithCov p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point[j] / voxel_size;       //~ voxel_size是每个voxel的尺寸，常规默认3m。这里是先获得索引。
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;    //~ 避免(int64_t)取整时错误
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);    //~ 查找这个Location
        auto iter = feat_map.find(position);        //~ feat_map 外部传入的就是VoxelMap，Hash映射的，VOXEL_LOC -> OctoTree
        if (iter != feat_map.end()) {               //~ 如果不是末尾，即找到了对应的映射，则直接插入。插入的方式是更新OctoTree
            feat_map[position]->UpdateOctoTree(p_v);
        } else {                                    //~ 否则，在指定位置创建一个新的OctoTree
            OctoTree *octo_tree = new OctoTree(max_layer, 0, layer_point_size, max_points_size, max_cov_points_size, planer_threshold);
            feat_map[position] = octo_tree;           //~ 建立新的映射；并计算这个OctoTree相关的参数。
            feat_map[position]->quater_length_ = voxel_size / 4;
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->UpdateOctoTree(p_v);  //~ 计算完成后，将这个点再插入到OctoTree中
        }
    }
}

void transformLidar(const StatesGroup &state,
                                        const shared_ptr<ImuProcess> &p_imu,
                                        const PointCloudXYZI::Ptr &input_cloud,
                                        pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud) {
    trans_cloud->clear();
    for (size_t i = 0; i < input_cloud->size(); i++) {
        pcl::PointXYZINormal p_c = input_cloud->points[i];
        Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
        // p = p_imu->Lid_rot_to_IMU * p + p_imu->Lid_offset_to_IMU;
        p = state.rot_end * p + state.pos_end;
        pcl::PointXYZI pi;
        pi.x = p(0);
        pi.y = p(1);
        pi.z = p(2);
        pi.intensity = p_c.intensity;
        trans_cloud->points.push_back(pi);
    }
}

void build_single_residual(const pointWithCov &pv, const OctoTree *current_octo,
                           const int current_layer, const int max_layer,
                           const double sigma_num, bool &is_sucess,
                           double &prob, ptpl &single_ptpl) {
    double radius_k = 3;
    Eigen::Vector3d p_w = pv.point_world;
    if (current_octo->plane_ptr_->is_plane) {
        Plane &plane = *current_octo->plane_ptr_;
        Eigen::Vector3d p_world_to_center = p_w - plane.center;
        double proj_x = p_world_to_center.dot(plane.x_normal);
        double proj_y = p_world_to_center.dot(plane.y_normal);
        float dis_to_plane =
                fabs(plane.normal(0) * p_w(0) + plane.normal(1) * p_w(1) +
             plane.normal(2) * p_w(2) + plane.d);
        float dis_to_center =
                (plane.center(0) - p_w(0)) * (plane.center(0) - p_w(0)) +
                (plane.center(1) - p_w(1)) * (plane.center(1) - p_w(1)) +
                (plane.center(2) - p_w(2)) * (plane.center(2) - p_w(2));
        float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

        if (range_dis <= radius_k * plane.radius) {
            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = p_w - plane.center;
            J_nq.block<1, 3>(0, 3) = -plane.normal;
            double sigma_l = J_nq * plane.plane_cov * J_nq.transpose();
            sigma_l += plane.normal.transpose() * pv.cov * plane.normal;
            if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
                is_sucess = true;
                double this_prob = 1.0 / (sqrt(sigma_l)) *
                           exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                if (this_prob > prob) {
                    prob = this_prob;
                    single_ptpl.point = pv.point;
                    single_ptpl.plane_cov = plane.plane_cov;
                    single_ptpl.normal = plane.normal;
                    single_ptpl.center = plane.center;
                    single_ptpl.d = plane.d;
                    single_ptpl.layer = current_layer;
                }
                return;
            } else {
                // is_sucess = false;
                return;
            }
        } else {
            // is_sucess = false;
            return;
        }
    } else {
        if (current_layer < max_layer) {
            for (size_t leafnum = 0; leafnum < 8; leafnum++) {
                if (current_octo->leaves_[leafnum] != nullptr) {

                    OctoTree *leaf_octo = current_octo->leaves_[leafnum];
                    build_single_residual(pv, leaf_octo, current_layer + 1, max_layer,
                                                                sigma_num, is_sucess, prob, single_ptpl);
                }
            }
            return;
        } else {
            // is_sucess = false;
            return;
        }
    }
}

void GetUpdatePlane(const OctoTree *current_octo, const int pub_max_voxel_layer,
                                        std::vector<Plane> &plane_list) {
    if (current_octo->layer_ > pub_max_voxel_layer) {
        return;
    }
    if (current_octo->plane_ptr_->is_update) {
        plane_list.push_back(*current_octo->plane_ptr_);
    }
    if (current_octo->layer_ < current_octo->max_layer_) {
        if (!current_octo->plane_ptr_->is_plane) {
            for (size_t i = 0; i < 8; i++) {
                if (current_octo->leaves_[i] != nullptr) {
                    GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer,
                         plane_list);
                }
            }
        }
    }
    return;
}

// void BuildResidualListTBB(const unordered_map<VOXEL_LOC, OctoTree *>
// &voxel_map,
//                           const double voxel_size, const double sigma_num,
//                           const int max_layer,
//                           const std::vector<pointWithCov> &pv_list,
//                           std::vector<ptpl> &ptpl_list,
//                           std::vector<Eigen::Vector3d> &non_match) {
//   std::mutex mylock;
//   ptpl_list.clear();
//   std::vector<ptpl> all_ptpl_list(pv_list.size());
//   std::vector<bool> useful_ptpl(pv_list.size());
//   std::vector<size_t> index(pv_list.size());
//   for (size_t i = 0; i < index.size(); ++i) {
//     index[i] = i;
//     useful_ptpl[i] = false;
//   }
//   std::for_each(
//       std::execution::par_unseq, index.begin(), index.end(),
//       [&](const size_t &i) {
//         pointWithCov pv = pv_list[i];
//         float loc_xyz[3];
//         for (int j = 0; j < 3; j++) {
//           loc_xyz[j] = pv.point_world[j] / voxel_size;
//           if (loc_xyz[j] < 0) {
//             loc_xyz[j] -= 1.0;
//           }
//         }
//         VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
//                            (int64_t)loc_xyz[2]);
//         auto iter = voxel_map.find(position);
//         if (iter != voxel_map.end()) {
//           OctoTree *current_octo = iter->second;
//           ptpl single_ptpl;
//           bool is_sucess = false;
//           double prob = 0;
//           build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
//                                 is_sucess, prob, single_ptpl);
//           if (!is_sucess) {
//             VOXEL_LOC near_position = position;
//             if (loc_xyz[0] > (current_octo->voxel_center_[0] +
//                               current_octo->quater_length_)) {
//               near_position.x = near_position.x + 1;
//             } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
//                                      current_octo->quater_length_)) {
//               near_position.x = near_position.x - 1;
//             }
//             if (loc_xyz[1] > (current_octo->voxel_center_[1] +
//                               current_octo->quater_length_)) {
//               near_position.y = near_position.y + 1;
//             } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
//                                      current_octo->quater_length_)) {
//               near_position.y = near_position.y - 1;
//             }
//             if (loc_xyz[2] > (current_octo->voxel_center_[2] +
//                               current_octo->quater_length_)) {
//               near_position.z = near_position.z + 1;
//             } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
//                                      current_octo->quater_length_)) {
//               near_position.z = near_position.z - 1;
//             }
//             auto iter_near = voxel_map.find(near_position);
//             if (iter_near != voxel_map.end()) {
//               build_single_residual(pv, iter_near->second, 0, max_layer,
//                                     sigma_num, is_sucess, prob, single_ptpl);
//             }
//           }
//           if (is_sucess) {

//             mylock.lock();
//             useful_ptpl[i] = true;
//             all_ptpl_list[i] = single_ptpl;
//             mylock.unlock();
//           } else {
//             mylock.lock();
//             useful_ptpl[i] = false;
//             mylock.unlock();
//           }
//         }
//       });
//   for (size_t i = 0; i < useful_ptpl.size(); i++) {
//     if (useful_ptpl[i]) {
//       ptpl_list.push_back(all_ptpl_list[i]);
//     }
//   }
// }

void BuildResidualListOMP(const unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                                                    const double voxel_size, 
                                                    const double sigma_num,
                                                    const int max_layer,
                                                    const std::vector<pointWithCov> &pv_list,
                                                    std::vector<ptpl> &ptpl_list,
                                                    std::vector<Eigen::Vector3d> &non_match) {
    std::mutex mylock;
    ptpl_list.clear();
    std::vector<ptpl> all_ptpl_list(pv_list.size());
    std::vector<bool> useful_ptpl(pv_list.size());
    std::vector<size_t> index(pv_list.size());
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
        useful_ptpl[i] = false;
    }
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < index.size(); i++) {
        pointWithCov pv = pv_list[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_world[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
        auto iter = voxel_map.find(position);
        if (iter != voxel_map.end()) {
            OctoTree *current_octo = iter->second;
            ptpl single_ptpl;
            bool is_sucess = false;
            double prob = 0;
            build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
                                                        is_sucess, prob, single_ptpl);
            if (!is_sucess) {
                VOXEL_LOC near_position = position;
                if (loc_xyz[0] >
                        (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
                                 current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] >
                        (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
                                 current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] >
                        (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
                                 current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = voxel_map.find(near_position);
                if (iter_near != voxel_map.end()) {
                    build_single_residual(pv, iter_near->second, 0, max_layer, sigma_num,
                                                                is_sucess, prob, single_ptpl);
                }
            }
            if (is_sucess) {

                mylock.lock();
                useful_ptpl[i] = true;
                all_ptpl_list[i] = single_ptpl;
                mylock.unlock();
            } else {
                mylock.lock();
                useful_ptpl[i] = false;
                mylock.unlock();
            }
        }
    }
    for (size_t i = 0; i < useful_ptpl.size(); i++) {
        if (useful_ptpl[i]) {
            ptpl_list.push_back(all_ptpl_list[i]);
        }
    }
}

void BuildResidualListNormal(
        const unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
        const double voxel_size, const double sigma_num, const int max_layer,
        const std::vector<pointWithCov> &pv_list, std::vector<ptpl> &ptpl_list,
        std::vector<Eigen::Vector3d> &non_match) {
    ptpl_list.clear();
    std::vector<size_t> index(pv_list.size());
    for (size_t i = 0; i < pv_list.size(); ++i) {
        pointWithCov pv = pv_list[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_world[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
        auto iter = voxel_map.find(position);
        if (iter != voxel_map.end()) {
            OctoTree *current_octo = iter->second;
            ptpl single_ptpl;
            bool is_sucess = false;
            double prob = 0;
            build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
                                                        is_sucess, prob, single_ptpl);

            if (!is_sucess) {
                VOXEL_LOC near_position = position;
                if (loc_xyz[0] >
                        (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
                                 current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] >
                        (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
                                 current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] >
                        (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
                                 current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = voxel_map.find(near_position);
                if (iter_near != voxel_map.end()) {
                    build_single_residual(pv, iter_near->second, 0, max_layer, sigma_num,
                                                                is_sucess, prob, single_ptpl);
                }
            }
            if (is_sucess) {
                ptpl_list.push_back(single_ptpl);
            } else {
                non_match.push_back(pv.point_world);
            }
        }
    }
}

void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q) {
    Eigen::Matrix3d rot;
    rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0), z_vec(1), z_vec(2);
    Eigen::Matrix3d rotation = rot.transpose();
    Eigen::Quaterniond eq(rotation);
    q.w = eq.w();
    q.x = eq.x();
    q.y = eq.y();
    q.z = eq.z();
    // double norm = q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
    // if(norm<0.99){
    //     ROS_INFO_STREAM("Norm: " << norm);
    //     ROS_INFO_STREAM("Rot matrix: " << rot);
    //     ROS_INFO_STREAM("x_vec: " << x_vec);
    //     ROS_INFO_STREAM("y_vec: " << y_vec);
    // }
}

void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const Plane &single_plane, const float alpha, const Eigen::Vector3d rgb) {

    visualization_msgs::Marker plane;
    plane.header.frame_id = "camera_init";
    plane.header.stamp = ros::Time();
    plane.ns = plane_ns;
    plane.id = single_plane.id;
    plane.type = visualization_msgs::Marker::CYLINDER;
    plane.action = visualization_msgs::Marker::ADD;
    plane.pose.position.x = single_plane.center[0];
    plane.pose.position.y = single_plane.center[1];
    plane.pose.position.z = single_plane.center[2];

    geometry_msgs::Quaternion q;
    CalcVectQuation(single_plane.x_normal, single_plane.y_normal, single_plane.normal, q);
    plane.pose.orientation = q;
    plane.scale.x = 3 * sqrt(single_plane.max_eigen_value);
    plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value);
    plane.scale.z = 2 * sqrt(single_plane.min_eigen_value);
    plane.color.a = alpha;
    plane.color.r = rgb(0);
    plane.color.g = rgb(1);
    plane.color.b = rgb(2);
    plane.lifetime = ros::Duration();
    plane_pub.markers.push_back(plane);
}

void pubNoPlaneMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map, const ros::Publisher &plane_map_pub) {
    int id = 0;
    ros::Rate loop(500);
    float use_alpha = 0.8;
    visualization_msgs::MarkerArray voxel_plane;
    voxel_plane.markers.reserve(1000000);
    for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++) {
        if (!iter->second->plane_ptr_->is_plane) {
            for (uint i = 0; i < 8; i++) {
                if (iter->second->leaves_[i] != nullptr) {
                    OctoTree *temp_octo_tree = iter->second->leaves_[i];
                    if (!temp_octo_tree->plane_ptr_->is_plane) {
                        for (uint j = 0; j < 8; j++) {
                            if (temp_octo_tree->leaves_[j] != nullptr) {
                                if (!temp_octo_tree->leaves_[j]->plane_ptr_->is_plane) {
                                    Eigen::Vector3d plane_rgb(1, 1, 1);
                                    pubSinglePlane(voxel_plane, "no_plane", *(temp_octo_tree->leaves_[j]->plane_ptr_), use_alpha, plane_rgb);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    plane_map_pub.publish(voxel_plane);
    loop.sleep();
}

void pubVoxelMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                 const int pub_max_voxel_layer,
                 const ros::Publisher &plane_map_pub) {
    double max_trace = 0.25;
    double pow_num = 0.2;
    ros::Rate loop(500);
    float use_alpha = 0.8;
    visualization_msgs::MarkerArray voxel_plane;
    voxel_plane.markers.reserve(1000000);
    std::vector<Plane> pub_plane_list;
    for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
        GetUpdatePlane(iter->second, pub_max_voxel_layer, pub_plane_list);
    }
    for (size_t i = 0; i < pub_plane_list.size(); i++) {
        V3D plane_cov = pub_plane_list[i].plane_cov.block<3, 3>(0, 0).diagonal();
        double trace = plane_cov.sum();
        if (trace >= max_trace) {
            trace = max_trace;
        }
        trace = trace * (1.0 / max_trace);
        trace = pow(trace, pow_num);
        uint8_t r, g, b;
        mapJet(trace, 0, 1, r, g, b);
        Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
        double alpha;
        if (pub_plane_list[i].is_plane) {
            alpha = use_alpha;
        } else {
            alpha = 0;          //~ cool, use "alpha" to ignore not-plane voxel
        }
        pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
    }
    plane_map_pub.publish(voxel_plane);
    loop.sleep();
}

void pubPlaneMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map,
                 const ros::Publisher &plane_map_pub) {
    OctoTree *current_octo = nullptr;

    double max_trace = 0.25;
    double pow_num = 0.2;
    ros::Rate loop(500);
    float use_alpha = 1.0;
    visualization_msgs::MarkerArray voxel_plane;
    voxel_plane.markers.reserve(1000000);

    for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++) {
        if (iter->second->plane_ptr_->is_update) {
            Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);

            V3D plane_cov =
                    iter->second->plane_ptr_->plane_cov.block<3, 3>(0, 0).diagonal();
            double trace = plane_cov.sum();
            if (trace >= max_trace) {
                trace = max_trace;
            }
            trace = trace * (1.0 / max_trace);
            trace = pow(trace, pow_num);
            uint8_t r, g, b;
            mapJet(trace, 0, 1, r, g, b);
            Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
            // Eigen::Vector3d plane_rgb(1, 0, 0);
            float alpha = 0.0;
            if (iter->second->plane_ptr_->is_plane) {
                alpha = use_alpha;
            } else {
                // std::cout << "delete plane" << std::endl;
            }
            // if (iter->second->update_enable_) {
            //   plane_rgb << 1, 0, 0;
            // } else {
            //   plane_rgb << 0, 0, 1;
            // }
            pubSinglePlane(voxel_plane, "plane", *(iter->second->plane_ptr_), alpha,
                     plane_rgb);

            iter->second->plane_ptr_->is_update = false;
        } else {
            for (uint i = 0; i < 8; i++) {
                if (iter->second->leaves_[i] != nullptr) {
                    if (iter->second->leaves_[i]->plane_ptr_->is_update) {
                        Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);

                        V3D plane_cov = iter->second->leaves_[i]
                                                                ->plane_ptr_->plane_cov.block<3, 3>(0, 0)
                                                                .diagonal();
                        double trace = plane_cov.sum();
                        if (trace >= max_trace) {
                            trace = max_trace;
                        }
                        trace = trace * (1.0 / max_trace);
                        // trace = (max_trace - trace) / max_trace;
                        trace = pow(trace, pow_num);
                        uint8_t r, g, b;
                        mapJet(trace, 0, 1, r, g, b);
                        Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
                        plane_rgb << 0, 1, 0;
                        // fabs(iter->second->leaves_[i]->plane_ptr_->normal[0]),
                        //     fabs(iter->second->leaves_[i]->plane_ptr_->normal[1]),
                        //     fabs(iter->second->leaves_[i]->plane_ptr_->normal[2]);
                        float alpha = 0.0;
                        if (iter->second->leaves_[i]->plane_ptr_->is_plane) {
                            alpha = use_alpha;
                        } else {
                            // std::cout << "delete plane" << std::endl;
                        }
                        pubSinglePlane(voxel_plane, "plane",
                           *(iter->second->leaves_[i]->plane_ptr_), alpha,
                           plane_rgb);
                        // loop.sleep();
                        iter->second->leaves_[i]->plane_ptr_->is_update = false;
                        // loop.sleep();
                    } else {
                        OctoTree *temp_octo_tree = iter->second->leaves_[i];
                        for (uint j = 0; j < 8; j++) {
                            if (temp_octo_tree->leaves_[j] != nullptr) {
                                if (temp_octo_tree->leaves_[j]->octo_state_ == 0 &&
                                        temp_octo_tree->leaves_[j]->plane_ptr_->is_update) {
                                    if (temp_octo_tree->leaves_[j]->plane_ptr_->is_plane) {
                                        // std::cout << "subsubplane" << std::endl;
                                        Eigen::Vector3d normal_rgb(0.0, 1.0, 0.0);
                                        V3D plane_cov =
                                                temp_octo_tree->leaves_[j]
                                                        ->plane_ptr_->plane_cov.block<3, 3>(0, 0)
                                                        .diagonal();
                                        double trace = plane_cov.sum();
                                        if (trace >= max_trace) {
                                            trace = max_trace;
                                        }
                                        trace = trace * (1.0 / max_trace);
                                        // trace = (max_trace - trace) / max_trace;
                                        trace = pow(trace, pow_num);
                                        uint8_t r, g, b;
                                        mapJet(trace, 0, 1, r, g, b);
                                        Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
                                        plane_rgb << 0, 0, 1;
                                        float alpha = 0.0;
                                        if (temp_octo_tree->leaves_[j]->plane_ptr_->is_plane) {
                                            alpha = use_alpha;
                                        }

                                        pubSinglePlane(voxel_plane, "plane",
                                   *(temp_octo_tree->leaves_[j]->plane_ptr_),
                                   alpha, plane_rgb);
                                        // loop.sleep();
                                        temp_octo_tree->leaves_[j]->plane_ptr_->is_update = false;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    plane_map_pub.publish(voxel_plane);
    // plane_map_pub.publish(voxel_norm);
    loop.sleep();
    // cout << "[Map Info] Plane counts:" << plane_count
    //      << " Sub Plane counts:" << sub_plane_count
    //      << " Sub Sub Plane counts:" << sub_sub_plane_count << endl;
    // cout << "[Map Info] Update plane counts:" << update_count
    //      << "total size: " << feat_map.size() << endl;
}

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov) {
    float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
    float range_var = range_inc * range_inc;
    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
            pow(sin(DEG2RAD(degree_inc)), 2);
    Eigen::Vector3d direction(pb);
    direction.normalize();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0,
            -direction(0), -direction(1), direction(0), 0;
    Eigen::Vector3d base_vector1(1, 1,
                               -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
            base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    cov = direction * range_var * direction.transpose() +
                A * direction_var * A.transpose();
};
