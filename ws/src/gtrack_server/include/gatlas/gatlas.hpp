/**
 * \file gatlas.hpp
 *
 */

#ifndef _GATLAS_HPP_
#define _GATLAS_HPP_ 

#include <unordered_map>
#include <set>
#include <mutex>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace gatlas {

/**
 * Data structure for the target
 * state.
 */
struct TargetData {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	int id;

	uint64_t timestamp; // Timestamp

	Eigen::Vector3d pos; // Position 
	Eigen::Vector3d vel; // Velocity

	int Nobservers; // Number of observers
};


/**
 * Transformation Data:
 * - Translation
 * - Rotation
 */
struct TransformData {
    Eigen::Quaterniond rot;
    Eigen::Vector3d t;

    TransformData() : 
        rot(Eigen::Quaterniond::Identity()),
        t(Eigen::Vector3d::Zero()) {}
};


/**
 * Element of the GAtlas:
 * It's a node containing the the node id and the 
 * map of all the connections (Transformations).
 */
struct GAtlasElement {
	// Current node
    int id;

	// Map of the nodes which has a relation with the current
	// node
    std::unordered_map<int, TransformData> relations; 
};

}

class GAtlas {
	public:
		GAtlas();
		~GAtlas();

		GAtlas(std::string& ip, int port);

		// Register the source of an Item
		void register_source(int id);
		// Unregister the source of an Item
		void unregister_source(int id);

		/**
		 * Insert target data into the atlas
		 */
		int update_target_data(int id,
				const Eigen::Vector3d& t,
				const Eigen::Vector3d& v,
				uint64_t timestamp = 0);

		/**
		 * Get target data from the atlas
		 */
		bool get_target_data(int id,
				Eigen::Vector3d& p,
				Eigen::Vector3d& v,
				uint64_t* timestamp);

		/**
		 * Insert map trasformation into the atlas
		 */
		void setTransform(int id_i, int id_j, 
				const gatlas::TransformData& TR);

		bool getTransform(int orig, int dest, gatlas::TransformData& tr);

		std::unordered_map<int, gatlas::TargetData> getMap();

		int get_items(std::vector<gatlas::TargetData>& vout);

		std::vector<int> get_fiducialsIds();

	private:
		/**
		 * Mutex
		 */
		std::mutex _atlasdata_mx;
		std::mutex _targetdata_mx;

		/**
		 * Data structure representing the GAtlas
		 */
        std::unordered_map<int, gatlas::GAtlasElement> _gatlas;

		/**
		 * Data structure representing the tracked objects 
		 */
		std::unordered_map<int, gatlas::TargetData> _targetdata; 

		/**
		 * Variable used during the searches in the GAtlas
		 */
        std::set<int> _visited;

		/**
		 * Find a path between two nodes in the GAtlas
		 */
        bool find_path(int s, int e, gatlas::TransformData& TR);
};
#endif
