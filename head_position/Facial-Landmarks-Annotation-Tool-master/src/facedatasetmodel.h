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

#ifndef FACEDATASETMODEL_H
#define FACEDATASETMODEL_H

#include "facedataset.h"

#include <QAbstractListModel>
#include <QList>
#include <QPixmap>

namespace ft
{
	/**
	 * Model class that encapsulates the access to the FaceDataset in order to provide data for
	 * the view classes in Qt (such as QListView).
	 */
	class FaceDatasetModel : public QAbstractListModel
	{
	public:
		/**
		 * Class constructor.
		 * @param pParent Instance of a QObject with the parent of the model. Default is NULL.
		 */
		FaceDatasetModel(QObject *pParent = NULL);

		/**
		 * Virtual class destructor.
		 */
		virtual ~FaceDatasetModel();

		/**
		 * Returns the number of rows under the given parent. When the parent is valid
		 * it means that rowCount is returning the number of children of parent.
		 * @param oParent A QModelIndex with the parent to be used in the query.
		 * @return Integer with the number of rows under the given parent.
		 */
		int rowCount(const QModelIndex &oParent = QModelIndex()) const;

		/**
		 * Returns the number of columns for the children of the given parent.
		 * @param oParent A QModelIndex with the parent to be used in the query.
		 * @return Integer with the number of columns for the children of the given parent.
		 */
		int columnCount(const QModelIndex &oParent = QModelIndex()) const;

		/**
		 * Returns the data stored under the given role for the item referred to by the index.
		 * @param oIndex A QModelIndex with the index of the data requested.
		 * @param iRole A DispleyRole identifying the role of the data requested (the default is DisplayRole).
		 * @return A QVariant with the data requested.
		 */
		QVariant data(const QModelIndex &oIndex, int iRole = Qt::DisplayRole) const;

		/**
		 * Returns the header data stored under the given section (column), for the given role.
		 * @param iSection Integer with the section (column) to retrieve the header data.
		 * @param eOrientation The orientation of the header (not used).
		 * @param iRole A DispleyRole identifying the role of the header data requested (the default is DisplayRole).
		 * @return A QVariant with the header data requested.
		 */
		QVariant headerData(int iSection, Qt::Orientation eOrientation, int iRole = Qt::DisplayRole) const;

		/**
		 * Updates the face data stored under the given role for the item referred by the index.
		 * @param oIndex A QModelIndex with the index of the data to be updated.
		 * param oValue A QVariant with the new value for the data under the index.
		 * @param iRole A DisplayRole identifying the role of the data to be updated.
		 * @return A boolean indicating if the update was successful (true) or not (false).
		 */
		bool setData(const QModelIndex &oIndex, const QVariant &oValue, int iRole = Qt::EditRole);

		/**
		 * Loads the data from the given text file in the YAML format
		 * (YAML Ain't Markup Language - http://en.wikipedia.org/wiki/YAML).
		 * @param sFileName QString with the name of the file to read the data from.
		 * @param sMsgError QString to receive the error message in case the method fails.
		 * @return Boolean indicating if the loading was successful (true) of failed (false).
		 */
		bool loadFromFile(const QString &sFileName, QString &sMsgError);

        /**
         * Saves the data to the given file in the YAML format
		 * (YAML Ain't Markup Language - http://en.wikipedia.org/wiki/YAML).
         * @param sFileName QString with the name of the file to write the data to.
		 * @param sMsgError QString to receive the error message in case the method fails.
		 * @return Boolean indicating if the saving was successful (true) of failed (false).
         */
        bool saveToFile(const QString &sFileName, QString &sMsgError) const;

		/**
		 * Adds the given images to the dataset.
		 * @param lImageFiles QStringList with the list of image file names to add.
		 * @return Boolean indicating if the operation was successful or not.
		 */
		bool addImages(const QStringList &lImageFiles);

		/**
		 * Removes the given images to the dataset.
		 * @param lImageFiles QList with the list of image indexes to remove.
		 * @return Boolean indicating if the operation was successful or not.
		 */
		bool removeImages(const QList<int> &lImageIndexes);

		/**
		 * Adds a new feature to the face dataset. A new feature is added to all
		 * face images in the dataset in the same coordinates.
		 * @param iID Integer with the identifier of the face feature.
		 * @param x Float with the x coordinate for the face features.
		 * @param y Float with the y coordinate for the face features.
		 */
		void addFeature(int iID, float x = 0.0f, float y = 0.0f);

		/**
		 * Removes the feature of given index in all face images.
		 * @param iIndex Index of the feature to remove.
		 */
		void removeFeature(const int iIndex);

		/**
		 * Connects the two given features in the face dataset.
		 * @param iIDSource Integer with the ID of the source feature.
		 * @param iIDTarget Integer with the ID of the target feature.
		 */
		void connectFeatures(int iIDSource, int iIDTarget);

		/**
		* Disconnects the two given features in the face dataset.
		* @param iIDSource Integer with the ID of the source feature.
		* @param iIDTarget Integer with the ID of the target feature.
		*/
		void disconnectFeatures(int iIDSource, int iIDTarget);

		/**
		 * Gets all features in the given face image index.
		 * @param iIndex Integer with the index of the face image to query.
		 * @return Vector of FaceFeature instances with the face features in the image.
		 */
		std::vector<FaceFeature*> getFeatures(const int iIndex);

		/**
		 * Queries the number of facial features in the dataset (applicable to all images).
		 * @return Integer with the number of face features in the dataset.
		 */
		int numFeatures() const;

	protected:

		/**
		 * Build a thumbnail for the given image index.
		 * @param iIndex Integer with the index of the image to build the thumbnail for.
		 * @return A QPixmap with the thumbmail. If the image could not be read,
		 * a thumbnail of the 'image missing' is returned instead.
		 */
		QPixmap buildThumbnail(const int iIndex);

		/**
		 * Queries the display/edit flags for the given index.
		 * @param oIndex A QModelIndex with the index to be queried.
		 * @return A Qt::ItemFlags with the flags to be applied to the given index.
		 */
		Qt::ItemFlags flags(const QModelIndex &oIndex) const;

	private:
		/** Instance of the face annotation dataset for data access. */
		FaceDataset *m_pFaceDataset;

		/**
		 * List of thumbnails for the existing images to improve access performance.
		 * The thumbnails are related to the images in the facedataset according to their
		 * indexed position.
		 */
		QList<QPixmap> m_lCachedThumbnails;
	};

}

#endif //FACEDATASETMODEL_H