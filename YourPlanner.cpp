#include "YourPlanner.h"
#include <rl/plan/SimpleModel.h>
#include <rl/plan/KdtreeNearestNeighbors.h>
#include <rl/plan/Viewer.h>
#include <iostream>

YourPlanner::YourPlanner() : RrtConConBase()
{
}

YourPlanner::~YourPlanner()
{
    this->sampler->setGoalBias(this->goal, this->start, 0.05);
}

::std::string
YourPlanner::getName() const
{
    return "Your Planner";
}

void YourPlanner::choose(::rl::math::Vector &chosen, int index)
{
    this->model->getDof();
    // your modifications here
    chosen = this->sampler->generate(index);
}

RrtConConBase::Neighbor
YourPlanner::nearest(const Tree &tree, const ::rl::math::Vector &chosen, int index)
{
    ::std::vector<NearestNeighbors::Neighbor> neighbors;
    if (index == 0)
        neighbors = this->nearestNeighbors0->nearest(Metric::Value(&chosen, Vertex()), 1);
    else
        neighbors = this->nearestNeighbors1->nearest(Metric::Value(&chosen, Vertex()), 1);
    return Neighbor(neighbors.front().second.second, this->model->inverseOfTransformedDistance(neighbors.front().first));
}

RrtConConBase::Vertex
YourPlanner::addVertex(Tree &tree, const ::rl::plan::VectorPtr &q, int index)
{
    Vertex v = ::boost::add_vertex(tree);
    tree[v].index = ::boost::num_vertices(tree) - 1;
    tree[v].q = q;

    if (index == 0)
    {
        this->nearestNeighbors0->push(Metric::Value(q.get(), v));
    }
    else
    {
        this->nearestNeighbors1->push(Metric::Value(q.get(), v));
    }

    if (NULL != this->viewer)
    {
        this->viewer->drawConfigurationVertex(*tree[v].q);
    }

    return v;
}

RrtConConBase::Vertex
YourPlanner::connect(Tree &tree, const Neighbor &nearest, const ::rl::math::Vector &chosen, int index)
{
    // Do first extend step

    ::rl::math::Real distance = nearest.second;
    ::rl::math::Real step = distance;

    bool reached = false;

    if (step <= this->delta)
    {
        reached = true;
    }
    else
    {
        step = this->delta;
    }

    ::rl::plan::VectorPtr last = ::std::make_shared<::rl::math::Vector>(this->model->getDof());

    // move "last" along the line q<->chosen by distance "step / distance"
    this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);

    this->model->setPosition(*last);
    this->model->updateFrames();

    if (this->model->isColliding())
    {
        tree[nearest.first].tmp += 1;
        return NULL;
    }

    ::rl::math::Vector next(this->model->getDof());

    while (!reached)
    {
        // Do further extend step

        distance = this->model->distance(*last, chosen);
        step = distance;

        if (step <= this->delta)
        {
            reached = true;
        }
        else
        {
            step = this->delta;
        }

        // move "next" along the line last<->chosen by distance "step / distance"
        this->model->interpolate(*last, chosen, step / distance, next);

        this->model->setPosition(next);
        this->model->updateFrames();

        if (this->model->isColliding())
        {
            break;
        }

        *last = next;
    }

    // "last" now points to the vertex where the connect step collided with the environment.
    // Add it to the tree
    Vertex connected = this->addVertex(tree, last, index);
    this->addEdge(nearest.first, connected, tree);
    return connected;
}

bool YourPlanner::solve()
{
    rl::plan::KdtreeNearestNeighbors nn0(this->model);
    rl::plan::KdtreeNearestNeighbors nn1(this->model);

    this->nearestNeighbors0 = &nn0;
    this->nearestNeighbors1 = &nn1;

    this->time = ::std::chrono::steady_clock::now();
    // Define the roots of both trees
    this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared<::rl::math::Vector>(*this->start), 0);
    this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared<::rl::math::Vector>(*this->goal), 1);

    Tree *a = &this->tree[0];
    Tree *b = &this->tree[1];

    int index_a = 0;
    int index_b = 1;
    ::rl::math::Vector chosen(this->model->getDof());

    while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
    {
        // First grow tree a and then try to connect b.
        // then swap roles: first grow tree b and connect to a.
        for (::std::size_t j = 0; j < 2; ++j)
        {
            // Sample a random configuration
            this->choose(chosen, index_a);

            // Find the nearest neighbour in the tree
            Neighbor aNearest = this->nearest(*a, chosen, index_a);

            // Do a CONNECT step from the nearest neighbour to the sample
            Vertex aConnected = this->connect(*a, aNearest, chosen, index_a);

            // If a new node was inserted tree a
            if (NULL != aConnected)
            {
                // Try a CONNECT step form the other tree to the sample
                Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q, index_b);
                Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q, index_b);

                if (NULL != bConnected)
                {
                    // Test if we could connect both trees with each other
                    if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q))
                    {
                        this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
                        this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
                        return true;
                    }
                }
            }

            // Swap the roles of a and b
            using ::std::swap;
            swap(a, b);
            swap(index_a, index_b);
        }
    }
    return false;
}
