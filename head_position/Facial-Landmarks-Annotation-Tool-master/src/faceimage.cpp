/*
 * Copyright (C) 2016 Luiz Carlos Vieira (http://www.luiz.vieira.nom.br)
 *
 * This file is part of FLAT.
 *
 * FLAT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FLAT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "faceimage.h"

#include <QApplication>

using namespace std;

// +-----------------------------------------------------------
ft::FaceImage::FaceImage(const QString &sFileName)
{
	m_sFileName = sFileName;
}

// +-----------------------------------------------------------
ft::FaceImage::~FaceImage()
{
	clear();
}

// +-----------------------------------------------------------
void ft::FaceImage::clear()
{
	foreach(FaceFeature *pFeature, m_vFeatures)
		delete pFeature;
	foreach(FaceFeatureEdge *pEdge, m_vConnections)
		delete pEdge;
	m_vFeatures.clear();
	m_vConnections.clear();
}

// +-----------------------------------------------------------
QString ft::FaceImage::fileName() const
{
	return m_sFileName;
}

// +-----------------------------------------------------------
void ft::FaceImage::setFileName(QString sFileName)
{
	m_sFileName = sFileName;
}

// +-----------------------------------------------------------
bool ft::FaceImage::loadFromXML(const QDomElement &oElement, QString &sMsgError, int iNumExpectedFeatures)
{
	// Check the element name
	if(oElement.tagName() != "Sample")
	{
		sMsgError = QString(QApplication::translate("FaceImage", "invalid node name [%1] - expected node '%2'").arg(oElement.tagName(), "Sample"));
		return false;
	}

	// Read the file name
	QString sFile = oElement.attribute("fileName");
	if(sFile == "")
	{
		sMsgError = QString(QApplication::translate("FaceImage", "the attribute '%1' does not exist or it contains an invalid value").arg("fileName"));
		return false;
	}

	// Read the face features
	// Sample images
	QDomElement oFeatures = oElement.firstChildElement("Features");
	if(oFeatures.isNull() || oFeatures.childNodes().count() != iNumExpectedFeatures)
	{
		sMsgError = QString(QApplication::translate("FaceImage", "the node '%1' does not exist or it contains less children nodes than expected").arg("Features"));
		return false;
	}

	vector<FaceFeature*> vFeatures;
	for(QDomElement oElement = oFeatures.firstChildElement(); !oElement.isNull(); oElement = oElement.nextSiblingElement())
	{
		FaceFeature *pFeature = new FaceFeature();
		if(!pFeature->loadFromXML(oElement, sMsgError))
		{
			foreach(FaceFeature *pFeat, vFeatures)
				delete(pFeat);
			delete pFeature;

			return false;
		}
		vFeatures.push_back(pFeature);
	}

	clear();
	m_sFileName = sFile;
	m_vFeatures = vFeatures;
	return true;
}

// +-----------------------------------------------------------
void ft::FaceImage::saveToXML(QDomElement &oParent) const
{
	// Add the "Sample" node
	QDomElement oSample = oParent.ownerDocument().createElement("Sample");
	oParent.appendChild(oSample);

	// Define it's attributes
	oSample.setAttribute("fileName", m_sFileName);

	// Add the "Features" subnode
	QDomElement oFeatures = oParent.ownerDocument().createElement("Features");
	oSample.appendChild(oFeatures);

	// Add the nodes for the features
	foreach(FaceFeature *pFeat, m_vFeatures)
		pFeat->saveToXML(oFeatures);
}

// +-----------------------------------------------------------
QPixmap ft::FaceImage::pixMap() const
{
	QPixmap oRet;
	oRet.load(m_sFileName);
	return oRet;
}

// +-----------------------------------------------------------
ft::FaceFeature* ft::FaceImage::addFeature(int iID, float x, float y)
{
	FaceFeature *pFeat = new FaceFeature(iID, x, y);
	m_vFeatures.push_back(pFeat);
	return pFeat;
}

// +-----------------------------------------------------------
ft::FaceFeature* ft::FaceImage::getFeature(const int iIndex) const
{
	if(iIndex < 0 || iIndex >= (int) m_vFeatures.size())
		return NULL;
	return m_vFeatures[iIndex];
}

// +-----------------------------------------------------------
std::vector<ft::FaceFeature*> ft::FaceImage::getFeatures() const
{
	return m_vFeatures;
}

// +-----------------------------------------------------------
bool ft::FaceImage::removeFeature(const int iIndex)
{
	if(iIndex < 0 || iIndex >= (int) m_vFeatures.size())
		return false;

	FaceFeature *pFeat = m_vFeatures[iIndex];
	m_vFeatures.erase(m_vFeatures.begin() + iIndex);
	delete pFeat;

	return true;
}

// +-----------------------------------------------------------
bool ft::FaceImage::connectFeatures(int iIDSource, int iIDTarget)
{
	if (iIDSource < 0 || iIDSource >= (int)m_vFeatures.size())
		return false;
	if (iIDTarget < 0 || iIDTarget >= (int)m_vFeatures.size())
		return false;

	FaceFeature *pSource = m_vFeatures[iIDSource];
	FaceFeature *pTarget = m_vFeatures[iIDTarget];
	pSource->connectTo(pTarget);
	return true;
}

// +-----------------------------------------------------------
bool ft::FaceImage::disconnectFeatures(int iIDSource, int iIDTarget)
{
	if (iIDSource < 0 || iIDSource >= (int)m_vFeatures.size())
		return false;
	if (iIDTarget < 0 || iIDTarget >= (int)m_vFeatures.size())
		return false;

	FaceFeature *pSource = m_vFeatures[iIDSource];
	FaceFeature *pTarget = m_vFeatures[iIDTarget];
	pSource->disconnectFrom(pTarget);
	return true;
}