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

#include "facedataset.h"

#include <QDebug>
#include <QFileInfo>
#include <QDir>
#include <QApplication>

using namespace std;

// +-----------------------------------------------------------
ft::FaceDataset::FaceDataset()
{
	m_iNumFeatures = 0;
}

// +-----------------------------------------------------------
ft::FaceDataset::~FaceDataset()
{
	clear();
}

// +-----------------------------------------------------------
int ft::FaceDataset::size() const
{
	return m_vSamples.size();
}

// +-----------------------------------------------------------
bool ft::FaceDataset::loadFromFile(const QString &sFileName, QString &sMsgError)
{
	/******************************************************
	 * Open and read the file
	 ******************************************************/
	QFile oFile(sFileName);
	if (!oFile.open(QFile::ReadOnly))
    {
		sMsgError = QString(QApplication::translate("FaceDataset", "it was not possible to read from file [%1]")).arg(sFileName);
        return false;
    }

	// Used to resolve the image file names relative to the saved file path
	QDir oBase(QFileInfo(sFileName).absolutePath());

	QTextStream oData(&oFile);
	QString sData = oData.readAll();
	oFile.close();

	/******************************************************
	 * Parse the xml document
	 ******************************************************/
	QDomDocument oDoc;
	QString sError;
	int iLine, iColumn;
	if(!oDoc.setContent(sData, true, &sError, &iLine, &iColumn))
	{
		sMsgError = QString(QApplication::translate("FaceDataset", "there is an error in the contents of the file [%1]: error [%2], line [%3], column [%4]")).arg(sFileName, sError, QString::number(iLine), QString::number(iColumn));
		return false;
	}

	// Root node
	QDomElement oRoot = oDoc.firstChildElement("FaceDataset");

	if(oRoot.isNull())
	{
		sMsgError = QString(QApplication::translate("FaceDataset", "there is an error in the contents of the file [%1]: the node '%2' does not exist")).arg(sFileName, "FaceDataset");
		return false;
	}

	int iNumFeats = oRoot.attribute("numberOfFeatures", "-1").toInt();
	if(iNumFeats < 0)
	{
		sMsgError = QString(QApplication::translate("FaceDataset", "there is an error in the contents of the file [%1]: the attribute '%2' does not exist or it contains an invalid value")).arg(sFileName, "numberOfFeatures");
		return false;
	}

	// Sample images
	QDomElement oSamples = oRoot.firstChildElement("Samples");
	if(oSamples.isNull())
	{
		sMsgError = QString(QApplication::translate("FaceDataset", "there is an error in the contents of the file [%1]: the node '%2' does not exist")).arg(sFileName, "Samples");
		return false;
	}

	vector<FaceImage*> vSamples;
	for(QDomElement oElement = oSamples.firstChildElement(); !oElement.isNull(); oElement = oElement.nextSiblingElement())
	{
		FaceImage *pSample = new FaceImage();
		if(!pSample->loadFromXML(oElement, sError, iNumFeats))
		{
			foreach(FaceImage *pSamp, vSamples)
				delete(pSamp);
			delete pSample;

			sMsgError = QString(QApplication::translate("FaceDataset", "there is an error in the contents of the file [%1]: %2")).arg(sFileName, sError);
			return false;
		}
		pSample->setFileName(oBase.absoluteFilePath(pSample->fileName()));
		vSamples.push_back(pSample);
	}

	clear();
	m_iNumFeatures = iNumFeats;
	m_vSamples = vSamples;

	return true;
}

// +-----------------------------------------------------------
bool ft::FaceDataset::saveToFile(const QString &sFileName, QString &sMsgError) const
{
	/******************************************************
	 * Create the xml document
	 ******************************************************/
	QDomDocument oDoc;

	// Processing instruction
	QDomProcessingInstruction oInstr = oDoc.createProcessingInstruction("xml", "version='1.0' encoding='UTF-8'");
	oDoc.appendChild(oInstr);

	// Root node
	QDomElement oRoot = oDoc.createElementNS("https://github.com/luigivieira/Facial-Landmarks-Annotation-Tool", "FaceDataset");
	oDoc.appendChild(oRoot);

	oRoot.setAttribute("numberOfFeatures", m_iNumFeatures);

	// Sample images
	QDomElement oSamples = oDoc.createElement("Samples");
	oRoot.appendChild(oSamples);

	// Used to make the image file names relative to the saved file path
	QDir oBase(QFileInfo(sFileName).absolutePath());
	QString sSave;

	foreach(FaceImage *pImage, m_vSamples)
	{
		sSave = pImage->fileName(); // Save the absolute path (it is still used in the editor)
		pImage->setFileName(oBase.relativeFilePath(pImage->fileName()));
		pImage->saveToXML(oSamples);
		pImage->setFileName(sSave);
	}

	/******************************************************
	 * Save the file
	 ******************************************************/
	QFile oFile(sFileName);
	if (!oFile.open(QFile::WriteOnly | QFile::Truncate))
    {
		sMsgError = QString(QApplication::translate("FaceDataset", "it was not possible to write to file [%1]")).arg(sFileName);
        return false;
    }

	QTextStream oData(&oFile);
	oDoc.save(oData, 4);
	oFile.close();

	return true;
}

// +-----------------------------------------------------------
void ft::FaceDataset::clear()
{
	foreach(FaceImage *pImage, m_vSamples)
		delete pImage;
	m_vSamples.clear();

	m_iNumFeatures = 0;
}

// +-----------------------------------------------------------
ft::FaceImage* ft::FaceDataset::getImage(const int iIndex) const
{
    if(iIndex < 0 || iIndex >= size())
		return NULL;

	return m_vSamples[iIndex];
}

// +-----------------------------------------------------------
ft::FaceImage* ft::FaceDataset::addImage(const QString &sFileName)
{
	foreach(FaceImage *pImage, m_vSamples)
		if(pImage->fileName() == sFileName)
			return pImage;

	FaceImage *pRet = new FaceImage(sFileName);
	m_vSamples.push_back(pRet);
	return pRet;
}

// +-----------------------------------------------------------
bool ft::FaceDataset::removeImage(const int iIndex)
{
	if(iIndex < 0 || iIndex >= size())
		return false;

	FaceImage *pImage = m_vSamples[iIndex];
	m_vSamples.erase(m_vSamples.begin() + iIndex);
	delete pImage;

	return true;
}

// +-----------------------------------------------------------
int ft::FaceDataset::numFeatures() const
{
	return m_iNumFeatures;
}

// +-----------------------------------------------------------
void ft::FaceDataset::setNumFeatures(int iNumFeats)
{
	m_iNumFeatures = iNumFeats;
}

// +-----------------------------------------------------------
void ft::FaceDataset::addFeature(int iID, float x, float y)
{
	FaceFeature *pFeat;
	foreach(FaceImage *pSample, m_vSamples)
		pFeat = pSample->addFeature(iID, x, y);
	m_iNumFeatures++;
}

// +-----------------------------------------------------------
bool ft::FaceDataset::removeFeature(const int iIndex)
{
	if(iIndex < 0 || iIndex >= m_iNumFeatures)
		return false;

	foreach(FaceImage *pSample, m_vSamples)
		pSample->removeFeature(iIndex);
	m_iNumFeatures--;

	return true;
}

// +-----------------------------------------------------------
bool ft::FaceDataset::connectFeatures(int iIDSource, int iIDTarget)
{
	foreach(FaceImage *pSample, m_vSamples)
		pSample->connectFeatures(iIDSource, iIDTarget);

	return true;
}

// +-----------------------------------------------------------
bool ft::FaceDataset::disconnectFeatures(int iIDSource, int iIDTarget)
{
	foreach(FaceImage *pSample, m_vSamples)
		pSample->disconnectFeatures(iIDSource, iIDTarget);

	return true;
}

// +-----------------------------------------------------------
vector<ft::FaceFeature*> ft::FaceDataset::getImageFeatures(const int iIndex)
{
	if(iIndex < 0 || iIndex >= (int) m_vSamples.size())
		return vector<FaceFeature*>();

	return m_vSamples[iIndex]->getFeatures();
}