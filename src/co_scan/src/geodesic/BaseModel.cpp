#include "BaseModel.h"
#include "Parameters.h"
//#include <windows.h> // dsy: comment out.
//#include "gl\gl.h" // dsy: comment out.
//#include "gl\glu.h" // dsy: comment out.
#include <float.h>
#include <iostream>
//#include "gl\glut.h"
//#pragma comment(lib, "OPENGL32.LIB") // dsy: comment out.
//#pragma comment(lib, "GLU32.LIB") // dsy: comment out.
#include <fstream>
#include <sstream>
#include <algorithm>
using namespace std;


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CBaseModel::CBaseModel(const string& filename) : m_filename(filename)
{
}
/*
void CBaseModel::Render() const
{	
	glPushMatrix();
	GLint shadeModel;
	glGetIntegerv(GL_SHADE_MODEL, &shadeModel);
	if (shadeModel == GL_SMOOTH)
	{
		for (int i = 0; i < GetNumOfFaces(); ++i)
		{
			glBegin(GL_POLYGON);
			for (int j = 0; j < 3; ++j)
			{			
				const CPoint3D &pt = Vert(Face(i)[j]);
				if (!m_NormalsToVerts.empty())
				{
					const CPoint3D &normal = Normal(Face(i)[j]);
					glNormal3f((float)normal.x, (float)normal.y, (float)normal.z);
				}
				glVertex3f((float)pt.x, (float)pt.y, (float)pt.z);
			}	
			glEnd();
		}	
	}
	else
	{
		for (int i = 0; i < GetNumOfFaces(); ++i)
		{
			glBegin(GL_POLYGON);
			CPoint3D normal = VectorCross(Vert(Face(i)[0]), Vert(Face(i)[1]), Vert(Face(i)[2]));
			normal.Normalize();
			glNormal3f((float)normal.x, (float)normal.y, (float)normal.z);
			for (int j = 0; j < 3; ++j)
			{			
				const CPoint3D &pt = Vert(Face(i)[j]);
				glVertex3f((float)pt.x, (float)pt.y, (float)pt.z);
			}	
			glEnd();
		}	
	}
	
	glPopMatrix();
}
*/
void CBaseModel::ComputeScaleAndNormals()
{
	if (m_Verts.empty())
		return;
	m_NormalsToVerts.resize(m_Verts.size(), CPoint3D(0, 0, 0));
	CPoint3D center(0, 0, 0);
	double sumArea(0);
	CPoint3D sumNormal(0, 0, 0);
	double deta(0);
	for (int i = 0; i < (int)m_Faces.size(); ++i)
	{
		CPoint3D normal = VectorCross(Vert(Face(i)[0]),
			Vert(Face(i)[1]),
			Vert(Face(i)[2]));
		double area = normal.Len();
		CPoint3D gravity3 = Vert(Face(i)[0]) +	Vert(Face(i)[1]) + Vert(Face(i)[2]);
		center += area * gravity3;
		sumArea += area;
		sumNormal += normal;
		deta += gravity3 ^ normal;
		normal.x /= area;
		normal.y /= area;
		normal.z /= area;
		for (int j = 0; j < 3; ++j)
		{
			m_NormalsToVerts[Face(i)[j]] += normal;
		}
	}
	center /= sumArea * 3;
	deta -= 3 * (center ^ sumNormal);
	if (true)//deta > 0)
	{
		for (int i = 0; i < GetNumOfVerts(); ++i)
		{
			if (fabs(m_NormalsToVerts[i].x)
				+ fabs(m_NormalsToVerts[i].y)
				+ fabs(m_NormalsToVerts[i].z) >= FLT_EPSILON)
			{					
				m_NormalsToVerts[i].Normalize();
			}
		}
	}
	else
	{
		for (int i = 0; i < GetNumOfFaces(); ++i)
		{
			int temp = m_Faces[i][0];
			m_Faces[i][0] = m_Faces[i][1];
			m_Faces[i][1] = temp;
		}
		for (int i = 0; i < GetNumOfVerts(); ++i)
		{
			if (fabs(m_NormalsToVerts[i].x)
				+ fabs(m_NormalsToVerts[i].y)
				+ fabs(m_NormalsToVerts[i].z) >= FLT_EPSILON)
			{					
				double len = m_NormalsToVerts[i].Len();
				m_NormalsToVerts[i].x /= -len;
				m_NormalsToVerts[i].y /= -len;
				m_NormalsToVerts[i].z /= -len;
			}
		}
	}

	CPoint3D ptUp(m_Verts[0]);
	CPoint3D ptDown(m_Verts[0]);
	for (int i = 1; i < GetNumOfVerts(); ++i)
	{
		if (m_Verts[i].x > ptUp.x)
			ptUp.x = m_Verts[i].x;
		else if (m_Verts[i].x < ptDown.x)
			ptDown.x = m_Verts[i].x;
		if (m_Verts[i].y > ptUp.y)
			ptUp.y = m_Verts[i].y;
		else if (m_Verts[i].y < ptDown.y)
			ptDown.y = m_Verts[i].y;
		if (m_Verts[i].z > ptUp.z)
			ptUp.z = m_Verts[i].z;
		else if (m_Verts[i].z < ptDown.z)
			ptDown.z = m_Verts[i].z;
	}	

	double maxEdgeLenOfBoundingBox = -1;
	if (ptUp.x - ptDown.x > maxEdgeLenOfBoundingBox)
		maxEdgeLenOfBoundingBox = ptUp.x - ptDown.x;
	if (ptUp.y - ptDown.y > maxEdgeLenOfBoundingBox)
		maxEdgeLenOfBoundingBox = ptUp.y - ptDown.y;
	if (ptUp.z - ptDown.z > maxEdgeLenOfBoundingBox)
		maxEdgeLenOfBoundingBox = ptUp.z - ptDown.z;
	m_scale = maxEdgeLenOfBoundingBox / 2;
	//m_center = center;
	//m_ptUp = ptUp;
	//m_ptDown = ptDown;
}

void CBaseModel::LoadModel()
{
	ReadFile(m_filename);
	ComputeScaleAndNormals();
}


string CBaseModel::GetFileShortNameWithoutExtension() const
{
	int pos = (int)m_filename.size() - 1;
	while (pos >= 0)
	{
		if (m_filename[pos] == '\\')
			break;
		--pos;
	}
	++pos;
	int pos2 = (int)m_filename.size() - 1;
	while (pos2 >= 0)
	{
		if (m_filename[pos2] == '.')
			break;
		--pos2;
	}
	string str(m_filename.substr(pos, pos2 - pos));
	return str;
}

string CBaseModel::GetFileShortName() const
{
	int pos = (int)m_filename.size() - 1;
	while (pos >= 0)
	{
		if (m_filename[pos] == '\\')
			break;
		--pos;
	}	
	++pos;
	string str(m_filename.substr(pos));
	return str;
}

string CBaseModel::GetFileFullName() const
{
	return m_filename;
}

void CBaseModel::ReadObjFile(const string& filename)
{
	ifstream in(filename.c_str());
	if (in.fail())
	{
		throw "fail to read file";
	}
	char buf[256];
	while (in.getline(buf, sizeof buf))
	{
		istringstream line(buf);
		string word;
		line >> word;
		if (word == "v")
		{
			CPoint3D pt;
			line >> pt.x;
			line >> pt.y;
			line >> pt.z;

			m_Verts.push_back(pt);
		}
		else if (word == "f")
		{
			CFace face;
			int tmp;
			vector<int> polygon;
			polygon.reserve(4);
			while (line >> tmp)
			{
				polygon.push_back(tmp);
				char tmpBuf[256];
				line.getline(tmpBuf, sizeof tmpBuf, ' ');
			}
			for (int j = 1; j < (int)polygon.size() - 1; ++j)
			{
				face[0] = polygon[0] - 1;
				face[1] = polygon[j] - 1;
				face[2] = polygon[j + 1] - 1;
				m_Faces.push_back(face);
			}
		}
		else
		{
			continue;
		}
	}
	//m_Verts.swap(vector<CPoint3D>(m_Verts)); // dsy: comment out.
	//m_Faces.swap(vector<CFace>(m_Faces)); // dsy: comment out.
	in.close();
}

void CBaseModel::ReadFile(const string& filename)
{
	int nDot = (int)filename.rfind('.');
	if (nDot == -1)
	{
		throw "File name doesn't contain a dot!";
	}
	string extension = filename.substr(nDot + 1);
	
	if (extension == "obj")
	{
		ReadObjFile(filename);
	}
	else if (extension == "off")
	{
		ReadOffFile(filename);
	}
	else if (extension == "m")
	{
		ReadMFile(filename);
	}
	else
	{
		throw "This format can't be handled!";
	}
}

void CBaseModel::ReadOffFile(const string& filename)
{
	ifstream in(filename.c_str());
	if (in.fail())
	{
		throw "fail to read file";
	}
	char buf[256];
	in.getline(buf, sizeof buf);
	int vertNum, faceNum, edgeNum;
	in >> vertNum >> faceNum >> edgeNum;

	for (int i = 0; i < vertNum; ++i)
	{
		CPoint3D pt;
		in >> pt.x;
		in >> pt.y;
		in >> pt.z;
		m_Verts.push_back(pt);
	}
	//m_Verts.swap(vector<CPoint3D>(m_Verts)); // dsy: comment out.
	
	int degree;
	while (in >> degree)
	{
		int first, second;
		in >> first >> second;

		for (int i = 0; i < degree - 2; ++i)
		{
			CFace f;
			f[0] = first;
			f[1] = second;
			in >> f[2];
			m_Faces.push_back(f);			
			second = f[2];
		}
	}

	in.close();
	//m_Faces.swap(vector<CFace>(m_Faces)); // dsy: comment out.
}

void CBaseModel::ReadMFile(const string& filename)
{
	ifstream in(filename.c_str());
	if (in.fail())
	{
		throw "fail to read file";
	}
	char buf[256];
	while (in.getline(buf, sizeof buf))
	{
		istringstream line(buf);
		if (buf[0] == '#')
			continue;
		string word;
		line >> word;
		if (word == "Vertex")
		{
			int tmp;
			line >> tmp;
			CPoint3D pt;
			line >> pt.x;
			line >> pt.y;
			line >> pt.z;

			m_Verts.push_back(pt);
		}
		else if (word == "Face")
		{
			CFace face;
			int tmp;
			line >> tmp;
			vector<int> polygon;
			polygon.reserve(4);
			while (line >> tmp)
			{
				polygon.push_back(tmp);
			}
			for (int j = 1; j < (int)polygon.size() - 1; ++j)
			{
				face[0] = polygon[0] - 1;
				face[1] = polygon[j] - 1;
				face[2] = polygon[j + 1] - 1;
				m_Faces.push_back(face);
			}
		}
		else
		{
			continue;
		}
	}
	//m_Verts.swap(vector<CPoint3D>(m_Verts)); // dsy: comment out.
	//m_Faces.swap(vector<CFace>(m_Faces)); // dsy: comment out.
	in.close();
}


void CBaseModel::SaveMFile(const string& filename) const
{
	ofstream outFile(filename.c_str());
	for (int i = 0; i < (int)GetNumOfVerts(); ++i)
	{
		outFile << "Vertex " << i + 1 << " " << Vert(i).x << " " << Vert(i).y << " " << Vert(i).z << endl;
	}
	int cnt(0);
	for (int i = 0; i < (int)GetNumOfFaces(); ++i)
	{
		if (m_UselessFaces.find(i) != m_UselessFaces.end())
			continue;
		outFile <<"Face " << ++cnt << " " << Face(i)[0] + 1 << " " << Face(i)[1] + 1 << " " << Face(i)[2] + 1 << endl;
	}
	outFile.close();
}

void CBaseModel::SaveOffFile(const string& filename) const
{
	ofstream outFile(filename.c_str());
	outFile << "OFF" << endl;
	outFile << GetNumOfVerts() << " " << GetNumOfFaces() << " " << 0 << endl;
	for (int i = 0; i < (int)GetNumOfVerts(); ++i)
	{
		outFile << Vert(i).x << " " << Vert(i).y << " " << Vert(i).z << endl;
	}
	for (int i = 0; i < (int)GetNumOfFaces(); ++i)
	{
		if (m_UselessFaces.find(i) != m_UselessFaces.end())
			continue;
		outFile << 3 << " " << Face(i)[0]<< " " << Face(i)[1] << " " << Face(i)[2] << endl;
	}
	outFile.close();
}

void CBaseModel::SaveObjFile(const string& filename) const
{
	ofstream outFile(filename.c_str());
	outFile << "g " << filename.substr(filename.rfind("\\") + 1, filename.rfind('.') - filename.rfind("\\") - 1) << endl;
	for (int i = 0; i < (int)GetNumOfVerts(); ++i)
	{
		outFile << "v " << Vert(i).x << " " << Vert(i).y << " " << Vert(i).z << endl;
	}
	for (int i = 0; i < (int)GetNumOfFaces(); ++i)
	{
		if (m_UselessFaces.find(i) != m_UselessFaces.end())
			continue;
		outFile << "f " << Face(i)[0] + 1 << " " << Face(i)[1] + 1<< " " << Face(i)[2] + 1<< endl;
	}
	outFile.close();
}

void CBaseModel::SaveScalarFieldObjFile(const vector<double>& vals, const string& filename) const
{
	ofstream outFile(filename.c_str());
	outFile << "g " << filename.substr(filename.rfind("\\") + 1, filename.rfind('.') - filename.rfind("\\") - 1) << endl;
	outFile << "# maxDis: " << *max_element(vals.begin(), vals.end()) << endl;
	for (int i = 0; i < (int)GetNumOfVerts(); ++i)
	{
		outFile << "v " << Vert(i).x << " " << Vert(i).y << " " << Vert(i).z << endl;
	}
	for (int i = 0; i < (int)vals.size(); ++i)
	{
		outFile << "vt " << vals[i] << " " << 0 << endl;
	}
	for (int i = 0; i < (int)GetNumOfFaces(); ++i)
	{
		if (m_UselessFaces.find(i) != m_UselessFaces.end())
			continue;
		outFile << "f " << Face(i)[0] + 1 << "/" << Face(i)[0] + 1
			<< " " << Face(i)[1] + 1 << "/" << Face(i)[1] + 1
			<< " " << Face(i)[2] + 1 << "/" << Face(i)[2] + 1 << endl;
	}
	outFile.close();
}

void CBaseModel::SaveScalarFieldObjFile(const vector<double>& vals, const string& comments, const string& filename) const
{
	ofstream outFile(filename.c_str());
	outFile << "g " << filename.substr(filename.rfind("\\") + 1, filename.rfind('.') - filename.rfind("\\") - 1) << endl;
	outFile << comments << endl;
	for (int i = 0; i < (int)GetNumOfVerts(); ++i)
	{
		outFile << "v " << Vert(i).x << " " << Vert(i).y << " " << Vert(i).z << endl;
	}
	for (int i = 0; i < (int)vals.size(); ++i)
	{
		outFile << "vt " << vals[i] << " " << 0 << endl;
	}
	for (int i = 0; i < (int)GetNumOfFaces(); ++i)
	{
		if (m_UselessFaces.find(i) != m_UselessFaces.end())
			continue;
		outFile << "f " << Face(i)[0] + 1 << "/" << Face(i)[0] + 1
			<< " " << Face(i)[1] + 1 << "/" << Face(i)[1] + 1
			<< " " << Face(i)[2] + 1 << "/" << Face(i)[2] + 1 << endl;
	}
	outFile.close();
}

void CBaseModel::SaveScalarFieldObjFile(const vector<double>& vals, double maxV, const string& filename) const
{
	ofstream outFile(filename.c_str());
	outFile << "g " << filename.substr(filename.rfind("\\") + 1, filename.rfind('.') - filename.rfind("\\") - 1) << endl;
	outFile << "# maxValue = " << *max_element(vals.begin(), vals.end()) / maxV << endl;

	for (int i = 0; i < (int)GetNumOfVerts(); ++i)
	{
		outFile << "v " << Vert(i).x << " " << Vert(i).y << " " << Vert(i).z << endl;
	}
	for (int i = 0; i < (int)vals.size(); ++i)
	{
		outFile << "vt " << vals[i] / maxV << " " << 0 << endl;
	}
	for (int i = 0; i < (int)GetNumOfFaces(); ++i)
	{
		if (m_UselessFaces.find(i) != m_UselessFaces.end())
			continue;
		outFile << "f " << Face(i)[0] + 1 << "/" << Face(i)[0] + 1
			<< " " << Face(i)[1] + 1 << "/" << Face(i)[1] + 1
			<< " " << Face(i)[2] + 1 << "/" << Face(i)[2] + 1 << endl;
	}
	outFile.close();
}

void CBaseModel::SavePamametrizationObjFile(const vector<pair<double, double>>& uvs, const string& filename) const
{
	ofstream outFile(filename.c_str());
	outFile << "g " << filename.substr(filename.rfind("\\") + 1, filename.rfind('.') - filename.rfind("\\") - 1) << endl;
	for (int i = 0; i < (int)GetNumOfVerts(); ++i)
	{
		outFile << "v " << Vert(i).x << " " << Vert(i).y << " " << Vert(i).z << endl;
	}
	for (int i = 0; i < (int)uvs.size(); ++i)
	{
		outFile << "vt " << uvs[i].first << " " << uvs[i].second << endl;
	}
	for (int i = 0; i < (int)GetNumOfFaces(); ++i)
	{
		if (m_UselessFaces.find(i) != m_UselessFaces.end())
			continue;
		outFile << "f " << Face(i)[0] + 1 << "/" << Face(i)[0] + 1
			<< " " << Face(i)[1] + 1 << "/" << Face(i)[1] + 1
			<< " " << Face(i)[2] + 1 << "/" << Face(i)[2] + 1 << endl;
	}
	outFile.close();
}

void CBaseModel::PrintInfo(ostream& out) const
{
	out << "Model info is as follows.\n";
	out << "Name: " << GetFileShortName() << endl;
	out << "VertNum = " << GetNumOfVerts() << endl;
	out << "FaceNum = " << GetNumOfFaces() << endl;
	out << "Scale = " << m_scale << endl;
}

CPoint3D CBaseModel::GetShiftVertex(int indexOfVert) const
{
	return Vert(indexOfVert) + Normal(indexOfVert) * RateOfNormalShift * GetScale();
}

//CPoint3D CBaseModel::ShiftVertex(int indexOfVert, double epsilon) const
//{
//	return Vert(indexOfVert) +  Normal(indexOfVert) * epsilon;
//}

int CBaseModel::GetVertexID(const CPoint3D& pt) const
{
	double dis = DBL_MAX;
	int id;
	for (int i = 0; i < GetNumOfVerts(); ++i)
	{
		if ((Vert(i) - pt).Len() < dis)
		{
			id = i;
			dis = (Vert(id)-pt).Len();
		}
	}
	return id;
}

string CBaseModel::GetComments(const char* filename)
{
	ifstream in(filename);
	char buf[256];
	string result;
	while (in.getline(buf, sizeof buf))
	{
		if (buf[0] == '#')
		{
			result += buf;
			result += "\n";
		}
	}
	in.close();
	return result;
}

vector<double> CBaseModel::GetScalarField(string filename)
{
	vector<double> scalarField;
	ifstream in(filename);
	char buf[256];
	while (in.getline(buf, sizeof buf))
	{
		istringstream line(buf);
		string word;
		line >> word;
		if (word == "vt")
		{
			double value;
			line >> value;
			scalarField.push_back(value);
		}
	}

	in.close();
	return scalarField;
}

void CBaseModel::SetFaces(const vector<CBaseModel::CFace>& faces)
{
	m_Faces = faces;
}

const vector<CBaseModel::CFace>& CBaseModel::GetFaces() const
{
	return m_Faces;
}