#pragma once
template <typename T>
class Vec3
{
	//public attributes
	public:
		//position
		T x, y, z;

	//public methods
	public:
		Vec3() {
			x = y = z = 0;
		};

		Vec3(T newX, T newY, T newZ) {
			x = newX;
			y = newY;
			z = newZ;
		};
		~Vec3() {};
};

//operator <<
template<typename T>
std::ostream &operator << (std::ostream &str, Vec3<T> const &point)
{
	return str << "Point x: " << point.x << " y: " << point.y << " z: " << point.z;
}

//operator == float
template<typename T>
bool operator == (Vec3<T> v1, T v2)
{
	return (v1.x == v2) && (v1.y == v2) && (v1.z == v2);
}

//operator == Vec3
template<typename T>
bool operator == (Vec3<T> v1, Vec3<T> v2)
{
	return (v1.x == v2.x) && (v1.y == v2.y) && (v1.z == v2.z);
}

//operator + float
template<typename T>
Vec3<T> operator + (Vec3<T> v1, T v2)
{
	v1.x += v2; 
	//v1.y += v2; 
	v1.z += v2;
	return v1;
}

//operator + Vec3
template<typename T>
Vec3<T> operator + (Vec3<T> v1, Vec3<T> v2)
{
	v1.x += v2.x;
	//v1.y += v2.y; 
	v1.z += v2.z;
	return v1;
}

//operator - float
template<typename T>
Vec3<T> operator - (Vec3<T> v1, T v2)
{
	v1.x -= v2;
	//v1.y -= v2; 
	v1.z -= v2;
	return v1;
}

//operator - Vec3
template<typename T>
Vec3<T> operator - (Vec3<T> v1, Vec3<T> v2)
{
	v1.x -= v2.x;
	//v1.y -= v2.y; 
	v1.z -= v2.z;
	return v1;
}

//operator / float
template<typename T>
Vec3<T> operator / (Vec3<T> v1, T v2)
{
	v1.x /= v2;
	//v1.y /= v2; 
	v1.z /= v2;
	return v1;
}

//operator / Vec3
template<typename T>
Vec3<T> operator / (Vec3<T> v1, Vec3<T> v2)
{
	v1.x /= v2.x;
	//v1.y /= v2.y; 
	v1.z /= v2.z;
	return v1;
}

//operator * float
template<typename T>
Vec3<T> operator * (Vec3<T> v1, T v2)
{
	v1.x *= v2;
	//v1.y *= v2; 
	v1.z *= v2;
	return v1;
}

//operator * Vec3
template<typename T>
Vec3<T> operator * (Vec3<T> v1, Vec3<T> v2)
{
	v1.x *= v2.x;
	//v1.y *= v2.y; 
	v1.z *= v2.z;
	return v1;
}

//define the class from the template, using float points
typedef Vec3<float> Vector3;