// BaseModel.h: interface for the CBaseModel class.
//
//////////////////////////////////////////////////////////////////////
#pragma once

#include "Point3D.h"
#include <string>
#include <vector>
#include <map>
#include <set>
using namespace std;

class CBaseModel  
{
public:	
	CBaseModel(const string& filename);
public:
	struct CFace
	{
		int verts[3];
		CFace(){}
		CFace(int x, int y, int z)
		{
			verts[0] = x;
			verts[1] = y;
			verts[2] = z;
		}
		int& operator[](int index)
		{
			return verts[index];
		}
		int operator[](int index) const
		{
			return verts[index];
		} 
	};	
	vector<CFace> m_Faces;
	vector<CPoint3D> m_Verts;
protected:
	vector<CPoint3D> m_NormalsToVerts;
	set<int> m_UselessFaces;
	string m_filename;	
	double m_scale;
	
protected:	
	void ReadMFile(const string& filename);
	void ReadFile(const string& filename);
	void ReadObjFile(const string& filename);
	void ReadOffFile(const string& filename);
	void ComputeScaleAndNormals();
public:
	virtual void LoadModel();
	int GetVertexID(const CPoint3D& pt) const;
	void SaveMFile(const string& filename) const;
	void SaveOffFile(const string& filename) const;
	void SaveObjFile(const string& filename) const;
	void SaveScalarFieldObjFile(const vector<double>& vals, const string& filename) const;
	void SaveScalarFieldObjFile(const vector<double>& vals, double maxV, const string& filename) const;
	void SaveScalarFieldObjFile(const vector<double>& vals, const string& comments, const string& filename) const;
	void SavePamametrizationObjFile(const vector<pair<double, double>>& uvs, const string& filename) const;
	static vector<double> GetScalarField(string filename);
	static string GetComments(const char* filename);
	virtual void PrintInfo(ostream& out) const;
//	virtual void Render() const;
	inline int GetNumOfVerts() const;
	inline int GetNumOfFaces() const;	
	string GetFileShortName() const;
	string GetFileShortNameWithoutExtension() const;
	string GetFileFullName() const; 
	inline const CPoint3D& Vert(int vertIndex) const;
	inline const CPoint3D& Normal(int vertIndex) const;
	inline const CFace& Face(int faceIndex) const;
	CPoint3D GetShiftVertex(int indexOfVert) const;
	//CPoint3D ShiftVertex(int indexOfVert, double epsilon) const;
	double GetScale() const {return m_scale;}
	void SetFaces(const vector<CBaseModel::CFace>& faces);
	const vector<CBaseModel::CFace>& GetFaces() const;
};

int CBaseModel::GetNumOfVerts() const
{
	return (int)m_Verts.size();
}

int CBaseModel::GetNumOfFaces() const
{
	return (int)m_Faces.size();
}

const CPoint3D& CBaseModel::Vert(int vertIndex) const
{
	return m_Verts[vertIndex];
}

const CPoint3D& CBaseModel::Normal(int vertIndex) const
{
	return m_NormalsToVerts[vertIndex];
}

const CBaseModel::CFace& CBaseModel::Face(int faceIndex) const
{
	return m_Faces[faceIndex];
}

