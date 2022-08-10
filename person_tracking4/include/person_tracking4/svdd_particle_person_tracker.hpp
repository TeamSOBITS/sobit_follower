#ifndef SVDD_PARTICLE_PERSON_TRACKER
#define SVDD_PARTICLE_PERSON_TRACKER

#include <person_tracking4/particle_person_tracker.hpp>
#include <svdd_leg_tracker_library/svdd_leg_tracker_library.hpp>

constexpr int SVDD_LEG_DETECTION= 4;

namespace person_tracking4 {
    class PersonList{
        public:
            PersonList( const double distance, const double angle ) :
                distance_(distance), angle_(angle) {}
            
            double distance_; 
            double angle_ ;

            static bool compareDistance(PersonList &a, PersonList &b) {
                return a.distance_ > b.distance_;   // from largest to smalles
            }
    };

    class SVDDParticlePersonTracker : public ParticlePersonTracker {
        private :
            ros::Publisher pub_leg_marker_;
            svdd_leg_tracker_library::SVDDLegTracker leg_tracker_;
            std::vector<person_tracking4::PersonList> svdd_results_;

            int detectLegsSVDD ( const PointCloud::Ptr laser, std::vector<svdd_leg_tracker_library::PersonInfo>* persons );
            bool detectPersonPositionSVDD ( const PointCloud::Ptr laser );
            int predictPersonPositionSVDD ( const PointCloud::Ptr laser );
                
        public :
            SVDDParticlePersonTracker ();
            int computeSVDDParticle ( 
                PointCloud::Ptr laser,  
                const std::vector<geometry_msgs::Point>& person_pts, 
                const ros::Time& stamp,
                person_tracking4::FollowPositionPtr follow_position );

            
    };
}

#endif