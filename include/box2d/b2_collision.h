// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef B2_COLLISION_H
#define B2_COLLISION_H

#include <limits.h>

#include "b2_api.h"
#include "b2_math.h"

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

// 类声明
class b2Shape;
class b2CircleShape;
class b2EdgeShape;
class b2PolygonShape;
// 定义特征的无效值
const uint8 b2_nullFeature = UCHAR_MAX;

/// The features that intersect to form the contact point 特征，交叉形成的接触点
/// This must be 4 bytes or less.    必须是4字节或者更少
struct B2_API b2ContactFeature
{
	enum Type
	{
		e_vertex = 0,
		e_face = 1
	};

	uint8 indexA;		///< Feature index on shapeA    A的特征索引
	uint8 indexB;		///< Feature index on shapeB    B的特征索引
	uint8 typeA;		///< The feature type on shapeA    A的特征类型
	uint8 typeB;		///< The feature type on shapeB    B的特征类型
};

/// Contact ids to facilitate warm starting.  接触ID
union B2_API b2ContactID
{
	b2ContactFeature cf;        /// 特征对象变量
	uint32 key;					/// 特征ID, 用于快速比较
};

/// A manifold point is a contact point belonging to a contact。  流形点属于接触流形的一个接触点，它具有的细节涉及到接触点的几何学和力学
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB   局部中心
/// -e_faceA: the local center of cirlceB or the clip point of polygonB    circleB的局部中心，或者polygonB的夹点
/// -e_faceB: the clip point of polygonA    polugonA的夹点
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
struct B2_API b2ManifoldPoint
{
	b2Vec2 localPoint;		/// 局部点，求解依赖于流形类型
	float normalImpulse;	/// 法向冲量，用于防止形状的穿透
	float tangentImpulse;	/// 切向冲量，用于模拟摩擦
	b2ContactID id;			/// 唯一地标识一个在两个形状之间的接触点
};

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
// 流形(注：也有人译为‘取样’，在物理和数学中均使用‘流形’，参照http://zh.wikipedia.org/wiki/流形 )
// 流形是两个凸形状的接触部分。
// Box2D支持多种类型的接触:
// 夹点与平面半径
// 点与点半径（圆）
// 局部的点求解取决于流形的类型：
// e_circles：circleA的中心
// e_faceA  : faceA的中心
// e_faceB  ：faceB的重心
// 同样局部法向量的求解：
// e_circles：不用
// e_faceA  : faceA的法向量
// e_faceB  ：faceB的法向量
// 我们用这种方式存储联系，以便移动时位置被更正。
// 所有接触场景必须表述为这些类型中的一个。
// 这个结构存储在时间步内，所以我们保持它小一些。
struct B2_API b2Manifold
{
	enum Type
	{
		e_circles,       // 圆
		e_faceA,         // 面A
		e_faceB          // 面B
	};

	b2ManifoldPoint points[b2_maxManifoldPoints];	/// 接触点数组
	b2Vec2 localNormal;								/// 局部法向量, 对Type::e_points没用
	b2Vec2 localPoint;								/// 局部点, 求解依赖流形类型
	Type type;                                      /// 类型
	int32 pointCount;								/// 流形的点的总数
};

/// This is used to compute the current state of a contact manifold.
/// 用于求解当下状态的接触流形
struct B2_API b2WorldManifold
{
	/// Evaluate the manifold with supplied transforms. This assumes
	/// modest motion from the original state. This does not change the
	/// point count, impulses, etc. The radii must come from the shapes
	/// that generated the manifold.
    /**************************************************************************
    * 功能描述：根据流形和提供的变换初始化此结构体。假设适度移动从原始状态开始的。
                这不能改变点的数量、冲量等等。半径必须来着与产生流形的形状。
    * 参数说明： manifold：流形的指针，用于初始化结构体
                 xfA     ：变换A的引用
                 radiusA ：形状A的半径
                 xfB     ：变化B的引用
                 radiusB ：形状B的半径
    * 返 回 值： (void)
    ***************************************************************************/
	void Initialize(const b2Manifold* manifold,
					const b2Transform& xfA, float radiusA,
					const b2Transform& xfB, float radiusB);

	b2Vec2 normal;								///< world vector pointing from A to B  世界向量方向从A到B
	b2Vec2 points[b2_maxManifoldPoints];		///< world contact point (point of intersection)   世界接触点(交点)
	float separations[b2_maxManifoldPoints];	///< a negative value indicates overlap, in meters 接触点重叠的距离
};

/// This is used for determining the state of contact points.
/// 定义接触点的状态
enum b2PointState
{
	b2_nullState,		///< point does not exist  无碰撞点
	b2_addState,		///< point was added in the update 在update中添加点
	b2_persistState,	///< point persisted across the update 在update中持续存在
	b2_removeState		///< point was removed in the update 点移除update
};

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
    /************************************************************************
     * 功能描述：通过两个流形计算点的状态。这些状态与从manifold1到maniflod2的过渡有关
     * 所以state1要么是持续更新要么就是删除，state2要么是添加要么是持续更新
     * 参数说明： state1   ：状态1，用于保存mainfold1中接触点的状态
                state2   ：状态2，用于保存mainfold2中接触点的状态
             manifold1：流形1
             manifold2：流形2
     * 返 回 值： (void)
     ***************************************************************************/
B2_API void b2GetPointStates(b2PointState state1[b2_maxManifoldPoints], b2PointState state2[b2_maxManifoldPoints],
					  const b2Manifold* manifold1, const b2Manifold* manifold2);

/// Used for computing contact manifolds.
/// 裁剪顶点结构体，用于接触流形的求解
struct B2_API b2ClipVertex
{
	b2Vec2 v;            // 接触点
	b2ContactID id;      // 接触ID
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
/// 光线输入数据。光线从p1扩展到p1 + maxFraction * (p2 - p1)
struct B2_API b2RayCastInput
{
	b2Vec2 p1, p2;       // 光线(或射线)上的两个点，其中p1是起始点
	float maxFraction;   // 需要检测的光线范围
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
/// 光线输出数据。光线达到p1 + fraction * (p2 - p1)，其中p1和 p2来自b2RayCastInput
struct B2_API b2RayCastOutput
{
	b2Vec2 normal;       // 法向量
	float fraction;      // 碰撞点位置的参数值
};

/// An axis aligned bounding box.
/// 轴对其包围盒
struct B2_API b2AABB
{
	/// Verify that the bounds are sorted.
	bool IsValid() const;

	/// Get the center of the AABB.
	b2Vec2 GetCenter() const
	{
		return 0.5f * (lowerBound + upperBound);
	}

	/// Get the extents of the AABB (half-widths).
	b2Vec2 GetExtents() const
	{
		return 0.5f * (upperBound - lowerBound);
	}

	/// Get the perimeter length
	float GetPerimeter() const
	{
		float wx = upperBound.x - lowerBound.x;
		float wy = upperBound.y - lowerBound.y;
		return 2.0f * (wx + wy);
	}

	/// Combine an AABB into this one.
	void Combine(const b2AABB& aabb)
	{
		lowerBound = b2Min(lowerBound, aabb.lowerBound);
		upperBound = b2Max(upperBound, aabb.upperBound);
	}

	/// Combine two AABBs into this one.
	void Combine(const b2AABB& aabb1, const b2AABB& aabb2)
	{
		lowerBound = b2Min(aabb1.lowerBound, aabb2.lowerBound);
		upperBound = b2Max(aabb1.upperBound, aabb2.upperBound);
	}

	/// Does this aabb contain the provided AABB.
	bool Contains(const b2AABB& aabb) const
	{
		bool result = true;
		result = result && lowerBound.x <= aabb.lowerBound.x;
		result = result && lowerBound.y <= aabb.lowerBound.y;
		result = result && aabb.upperBound.x <= upperBound.x;
		result = result && aabb.upperBound.y <= upperBound.y;
		return result;
	}

	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const;

	b2Vec2 lowerBound;	///< the lower vertex
	b2Vec2 upperBound;	///< the upper vertex
};

/// Compute the collision manifold between two circles.
    /**************************************************************************
    * 功能描述：求两个圆形成的碰撞流形
    * 参数说明： manifold ：流形对象的指针
                 circleA  ：圆形A对象指针
                 xfA        ：变换A对象引用
                 circleB  ：圆形B对象指针
                 xfB       ：变换B对象引用
    * 返 回 值： (void)
     ***************************************************************************/
B2_API void b2CollideCircles(b2Manifold* manifold,
					  const b2CircleShape* circleA, const b2Transform& xfA,
					  const b2CircleShape* circleB, const b2Transform& xfB);

/// Compute the collision manifold between a polygon and a circle.
    /**************************************************************************
     功能描述：求一个多边形和一个圆形成的碰撞流形
     参数说明： manifold ：流形对象的指针
             polygonA ：多边形A对象指针
             xfA      ：变换A对象引用
             circleB  ：圆形B对象指针
             xfB      ：变换B对象引用
     返 回 值： (void)
    **************************************************************************/
B2_API void b2CollidePolygonAndCircle(b2Manifold* manifold,
							   const b2PolygonShape* polygonA, const b2Transform& xfA,
							   const b2CircleShape* circleB, const b2Transform& xfB);

/// Compute the collision manifold between two polygons.
    /**************************************************************************
    * 功能描述：求解两个多边形碰撞产生的流形
    * 参数说明： manifold：碰撞流形指针，用于保存两个圆产生的流形
             polygonA：多边形A指针
               xfA     ：变换A
             polygonB：多边形B指针
               xfB     ：变换B
    * 返 回 值： (void)
    ***************************************************************************/
B2_API void b2CollidePolygons(b2Manifold* manifold,
					   const b2PolygonShape* polygonA, const b2Transform& xfA,
					   const b2PolygonShape* polygonB, const b2Transform& xfB);

/// Compute the collision manifold between an edge and a circle.
    /**************************************************************************
    * 功能描述：求解一个边缘形状和一个圆碰撞产生的流形
    * 参数说明： manifold：碰撞流形指针，用于保存两个圆产生的流形
                polygonA：多边形A指针
                xfA     ：变换A
                polygonB：多边形B指针
                xfB     ：变换B
    * 返 回 值： (void)
    ***************************************************************************/
B2_API void b2CollideEdgeAndCircle(b2Manifold* manifold,
							   const b2EdgeShape* polygonA, const b2Transform& xfA,
							   const b2CircleShape* circleB, const b2Transform& xfB);

/// Compute the collision manifold between an edge and a polygon.
    /**************************************************************************
    * 功能描述：求解一个边缘形状和一个多边形碰撞产生的流形
    * 参数说明： manifold：碰撞流形指针，用于保存两个圆产生的流形
                edgeA   ：边缘形状A指针
                xfA     ：变换A
                polygonB：多边形B指针
                xfB     ：变换B
    * 返 回 值： (void)
    ***************************************************************************/
B2_API void b2CollideEdgeAndPolygon(b2Manifold* manifold,
							   const b2EdgeShape* edgeA, const b2Transform& xfA,
							   const b2PolygonShape* circleB, const b2Transform& xfB);

/// Clipping for contact manifolds.
    /**************************************************************************
    * 功能描述：裁剪碰撞流形
    * 参数说明： vOut        ：裁剪顶点输出数组
                vIn         ：裁剪顶点输入数组
                normal      ：法向量
                offset      ：偏移量
                vertexIndexA：顶点索引
    * 返 回 值： 输出顶点的个数
    ***************************************************************************/
B2_API int32 b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
							const b2Vec2& normal, float offset, int32 vertexIndexA);

/// Determine if two generic shapes overlap.
    /**************************************************************************
    * 功能描述：测试两个通用的形状是否重叠。
                通过距离【Distance】判断是否重叠
    * 参数说明： shapeA ：形状A
              indexA ：索引A
              shapeB ：形状B
              indexB ：索引B
              xfA    ：变换A
              xfB    : 变换B
    * 返 回 值：true    ：重叠
            false   ：不重叠
     ***************************************************************************/
B2_API bool b2TestOverlap(	const b2Shape* shapeA, int32 indexA,
					const b2Shape* shapeB, int32 indexB,
					const b2Transform& xfA, const b2Transform& xfB);

// ---------------- Inline Functions ------------------------------------------

// 验证边界排序是否有效
inline bool b2AABB::IsValid() const
{
	b2Vec2 d = upperBound - lowerBound;
	bool valid = d.x >= 0.0f && d.y >= 0.0f;
	valid = valid && lowerBound.IsValid() && upperBound.IsValid();
	return valid;
}

/**************************************************************************
* 功能描述：测试两个通用的形状是否重叠。
            通过aabb判断是否重叠
* 参数说明： a ：AABB对象的引用
             b ：AABB对象的引用
* 返 回 值： true ：重叠
             false：不重叠
***************************************************************************/
inline bool b2TestOverlap(const b2AABB& a, const b2AABB& b)
{
	b2Vec2 d1, d2;
	d1 = b.lowerBound - a.upperBound;
	d2 = a.lowerBound - b.upperBound;

	if (d1.x > 0.0f || d1.y > 0.0f)
		return false;

	if (d2.x > 0.0f || d2.y > 0.0f)
		return false;

	return true;
}

#endif
