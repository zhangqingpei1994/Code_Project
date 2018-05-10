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
 
#include "facedatasetmodel.h"
#include <assert.h>

#include <QFileInfo>
#include <QApplication>
#include <QDebug>

// +-----------------------------------------------------------
ft::FaceDatasetModel::FaceDatasetModel(QObject *pParent):
	QAbstractListModel(pParent)
{
	m_pFaceDataset = new FaceDataset();
}

// +-----------------------------------------------------------
ft::FaceDatasetModel::~FaceDatasetModel()
{
	delete m_pFaceDataset;
}


// +-----------------------------------------------------------
int ft::FaceDatasetModel::rowCount(const QModelIndex &oParent) const
{
	Q_UNUSED(oParent);
	return m_pFaceDataset->size();
}

// +-----------------------------------------------------------
int ft::FaceDatasetModel::columnCount(const QModelIndex &oParent) const
{
	Q_UNUSED(oParent);
	return 4; // There are four columns of data: [Image Name]
}

// +-----------------------------------------------------------
QVariant ft::FaceDatasetModel::headerData(int iSection, Qt::Orientation eOrientation, int iRole) const
{
	Q_UNUSED(eOrientation);
	switch(iRole)
	{
		// Display data (the dataset contents)
		case Qt::DisplayRole:
			switch(iSection)
			{
				case 0: // [Image Name]
					return QApplication::translate("FaceDatasetModel", "Image Name");

				case 1: // [File Path]
					return QApplication::translate("FaceDatasetModel", "File Path");

				default:
					return QVariant();
			}
			

		case Qt::DecorationRole:
			if(iSection == 0)
				return QApplication::translate("FaceDatasetModel", "Thumbnail");
			else
				return QVariant();

		default:
			return QVariant();
	}
}

// +-----------------------------------------------------------
QVariant ft::FaceDatasetModel::data(const QModelIndex &oIndex, int iRole) const
{
	FaceImage *pImage = m_pFaceDataset->getImage(oIndex.row());
	if(!pImage)
		return QVariant();

	QPixmap oPixmap;

	switch(iRole)
	{
		// Data to be displayed on ModelViews
		case Qt::DisplayRole:
			switch(oIndex.column())
			{
				case 0: // [Image Name]
					return QFileInfo(pImage->fileName()).baseName();

				case 1: // [File Path]
					return pImage->fileName();

				default:
					return QVariant();
			}

		// Decoration data (image thumbnails for the first column)
		case Qt::DecorationRole:
			if(oIndex.column() == 0)
				return m_lCachedThumbnails[oIndex.row()];
			else
				return QVariant();

		// Raw data from the face dataset
		case Qt::UserRole:
			switch(oIndex.column())
			{
				case 0: // The complete image file name+path
					return pImage->fileName();

				case 2: // The image data
					oPixmap = pImage->pixMap();
					if(oPixmap.isNull())
						oPixmap = QPixmap(":/images/brokenimage");
					return oPixmap;

				default:
					return QVariant();
			}

		// Any other role, return invalid
		default:
			return QVariant();
	}
}

// +-----------------------------------------------------------
bool ft::FaceDatasetModel::setData(const QModelIndex &oIndex, const QVariant &oValue, int iRole)
{
	FaceImage *pImage = m_pFaceDataset->getImage(oIndex.row());
	if(!pImage)
		return false;

	if(iRole == Qt::UserRole)
	{
		switch(oIndex.column())
		{
			case 0: // [Image Name]
				return false;

			default:
				return false;
		}
	}
	else
		return false;
}

// +-----------------------------------------------------------
bool ft::FaceDatasetModel::loadFromFile(const QString &sFileName, QString &sMsgError)
{
	beginResetModel();
	bool bRet = m_pFaceDataset->loadFromFile(sFileName, sMsgError);

	// Rebuild the thumbnails cache
	if(bRet)
	{
		m_lCachedThumbnails.clear();
		for(int i = 0; i < m_pFaceDataset->size(); i++)
			m_lCachedThumbnails.append(buildThumbnail(i));
	}

	endResetModel();
	return bRet;
}

// +-----------------------------------------------------------
bool ft::FaceDatasetModel::saveToFile(const QString &sFileName, QString &sMsgError) const
{
	return m_pFaceDataset->saveToFile(sFileName, sMsgError);
}

// +-----------------------------------------------------------
bool ft::FaceDatasetModel::addImages(const QStringList &lImageFiles)
{
	int iFirst = m_pFaceDataset->size();
	int iLast = iFirst + lImageFiles.size() - 1;
	beginInsertRows(QModelIndex(), iFirst, iLast);
	for(int i = 0; i < lImageFiles.size(); i++)
	{
		m_pFaceDataset->addImage(lImageFiles[i]);
		m_lCachedThumbnails.append(buildThumbnail(m_pFaceDataset->size() - 1));
	}
	endInsertRows();
	emit dataChanged(index(iFirst), index(iLast));
	return true;
}

// +-----------------------------------------------------------
bool ft::FaceDatasetModel::removeImages(const QList<int> &lImageIndexes)
{
	int iFirst = lImageIndexes.first();
	int iLast = lImageIndexes.last();
	beginRemoveRows(QModelIndex(), iFirst, iLast);
	for(int i = lImageIndexes.size() - 1; i >= 0; i--)
	{
		m_pFaceDataset->removeImage(lImageIndexes[i]);
		m_lCachedThumbnails.removeAt(lImageIndexes[i]);
	}
	endRemoveRows();
	emit dataChanged(index(0), index(lImageIndexes.size() - 1));
	return true;
}

// +-----------------------------------------------------------
void ft::FaceDatasetModel::addFeature(int iID, float x, float y)
{
	m_pFaceDataset->addFeature(iID, x, y);
}

// +-----------------------------------------------------------
void ft::FaceDatasetModel::removeFeature(const int iIndex)
{
	m_pFaceDataset->removeFeature(iIndex);
}

// +-----------------------------------------------------------
void ft::FaceDatasetModel::connectFeatures(int iIDSource, int iIDTarget)
{
	m_pFaceDataset->connectFeatures(iIDSource, iIDTarget);
}

// +-----------------------------------------------------------
void ft::FaceDatasetModel::disconnectFeatures(int iIDSource, int iIDTarget)
{
	m_pFaceDataset->disconnectFeatures(iIDSource, iIDTarget);
}

// +-----------------------------------------------------------
std::vector<ft::FaceFeature*> ft::FaceDatasetModel::getFeatures(const int iIndex)
{
	return m_pFaceDataset->getImageFeatures(iIndex);
}

// +-----------------------------------------------------------
int ft::FaceDatasetModel::numFeatures() const
{
	return m_pFaceDataset->numFeatures();
}

// +-----------------------------------------------------------
QPixmap ft::FaceDatasetModel::buildThumbnail(const int iIndex)
{
	QPixmap oImage;
	FaceImage *pImage = m_pFaceDataset->getImage(iIndex);
	if(!pImage)
		oImage = QPixmap(":/images/imagemissing");
	else
	{
		oImage = pImage->pixMap();
		if(oImage.isNull())
			oImage = QPixmap(":/images/imagemissing");
	}

	oImage = oImage.scaled(50, 50, Qt::IgnoreAspectRatio);
	return oImage;
}

// +-----------------------------------------------------------
Qt::ItemFlags ft::FaceDatasetModel::flags(const QModelIndex &oIndex) const
{
	switch(oIndex.column())
	{
		case 0: // [Image Name]
			return Qt::ItemIsEnabled | Qt::ItemIsSelectable;

		default:
			return Qt::NoItemFlags;
	}
}
