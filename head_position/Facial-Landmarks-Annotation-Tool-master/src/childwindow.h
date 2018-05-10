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

#ifndef CHILDWINDOW_H
#define CHILDWINDOW_H

#include "facedatasetmodel.h"
#include "facewidget.h"

#include <QtGui>
#include <QWidget>

namespace ft
{
	/**
	 * MDI-child window class used to display and edit the face annotation datasets.
	 */
	class ChildWindow : public QWidget
	{
		Q_OBJECT
	public:
		/**
		 * Class constructor.
		 * @param pParent Instance of the widget that will be the parent of this window.
		 */
		explicit ChildWindow(QWidget* pParent = 0);

		/**
		 * Class destructor.
		 */
		virtual ~ChildWindow();

		/**
		 * Gets the list model used to display information about the contents of the
		 * face annotation dataset handled by this window.
		 * @return An instance of a QAbstractListModel that can be used with any subclass of QAbstractItemView
		 * to display lists, icons or trees with the face annotation dataset contents.
		 */
		FaceDatasetModel* dataModel() const;

		/**
		 * Gets the list selection model used to display selection information about the contents of the
		 * face annotation dataset handled by this window.
		 * @return An instance of a QItemSelectionModel that can be used with any subclass of QAbstractItemView
		 * to display lists, icons or tress with the face annotation dataset contents.
		 */
		QItemSelectionModel* selectionModel() const;

		/**
		 * Saves the contents of the face annotation dataset in this window to the current file
		 * (stored in the windowFilePath property). The file is saved in the YAML format, as defined
		 * in the FaceDataset class.
		 */
		bool save(QString &sMsgError);

		/**
		 * Saves the contents of the face annotation dataset in this window to the given file.
		 * The file is saved in the YAML format, as defined in the FaceDataset class.
		 * @param sFileName QString with the path and name of the file to save the dataset to.
		 */
		bool saveToFile(const QString &sFileName, QString &sMsgError);

		/**
		 * Loads the contents of the face annotation dataset from the given file into this window.
		 * The file must be in the YAML format, as defined in the FaceDataset class.
		 * @param sFileName QString with the path and name of the file to load the dataset from.
		 */
		bool loadFromFile(const QString &sFileName, QString &sMsgError);

		/**
		 * Sets the zoom level of the image in display in terms of the steps defined in the zoom
		 * slider from 1 to 21, with 11 (the middle value) as "no zoom" (i.e. 100% view).
		 * @param iLevel Integer with the zoom level to set.
		 */
		void setZoomLevel(const int iLevel);

		/**
		 * Gets the current zoom level of the image in display in terms of the steps defined in the zoom
		 * slider from 1 to 21, with 11 (the middle value) as "no zoom" (i.e. 100% view).
		 * @return Integer with the current zoom level.
		 */
		int getZoomLevel() const;

		/**
		 * Performs one step of zoom in.
		 */
		void zoomIn();

		/**
		 * Performs one step of zoom out.
		 */
		void zoomOut();

		/**
		 * Indicates if the face feature nodes are on display.
		 * @return Boolean indicating if the face feature nodes are being displayed or not.
		 */
		bool displayFaceFeatures() const;

		/**
		 * Updates the indication on if the face feature nodes shall be displayed or not.
		 * @param bValue Boolean with the new value (true means show, false means hide).
		 */
		void setDisplayFaceFeatures(const bool bValue);

		/**
		 * Indicates if the face feature edges are on display.
		 * @return Boolean indicating if the face feature edges are being displayed or not.
		 */
		bool displayConnections() const;

		/**
		 * Updates the indication on if the face feature edges shall be displayed or not.
		 * @param bValue Boolean with the new value (true means show, false means hide).
		 */
		void setDisplayConnections(const bool bValue);

		/**
		 * Indicates if the identifiers of the face feature nodes are on display.
		 * @return Boolean indicating if the identifiers of the face feature nodes are being displayed or not.
		 */
		bool displayFeatureIDs() const;

		/**
		 * Updates the indication on if the identifiers of the face feature nodes shall be displayed or not.
		 * @param bValue Boolean with the new value (true means show, false means hide).
		 */
		void setDisplayFeatureIDs(const bool bValue);

		/**
		 * Queries the list of existing face feature nodes.
		 * @param Const reference to the QList of existing nodes.
		 */
		const QList<FaceFeatureNode*>& getFaceFeatures() const;

		/**
		 * Queries the selected face feature nodes.
		 * @return QList with the pointers to the selected face feature nodes.
		 */
		QList<FaceFeatureNode*> getSelectedFeatures() const;

		/**
		 * Queries the selected face feature edges.
		 * @return QList with the pointers to the selected face feature edges.
		 */
		QList<FaceFeatureEdge*> getSelectedConnections() const;

		/**
		 * Sets the menu to be displayed upon events of context menu on the face features editor.
		 * The actions used in the menu must be controlled by the caller.
		 * @param pMenu Instance of the QMenu to be used for the context of the editor.
		 */
		void setContextMenu(QMenu *pMenu);

		/**
		 * Adds a new feature to the face feature editor, in the given position.
		 * @param oPos QPoint with the position (x, y) of the new feature.
		 */
		void addFeature(const QPoint &oPos);

		/**
		 * Removes the selected features and all their connections.
		 */
		void removeSelectedFeatures();

		/**
		 * Connects the selected features among themselves.
		 */
		void connectFeatures();

		/**
		 * Disconnects the selected features.
		 */
		void disconnectFeatures();

		/**
		 * Moves the face features in the image currently on display according to the given
		 * list of positions. If the number of face features is different than the number of
		 * points, the model is updated accordingly (with features added or removed to match
		 * the number of points).
		 * @param vPoints A std::vector with the list of QPoint instances with the new
		 * features' positions.
		 * @return Boolean indicating if the reposition was successfully done or not.
		 */
		bool positionFeatures(std::vector<QPoint> vPoints);

	protected:

		/**
		 * Refreshes the positions of face features in the editor based on the values in the dataset.
		 */
		void refreshFeaturesInWidget();

		/**
		 * Updates the positions of face features in the dataset based on the values in the editor.
		 */
		void updateFeaturesInDataset();

	protected slots:

		/**
		 * Captures the indication of changes in the image scale factor (zoom).
		 * @param dScaleFactor Double with the new scale factor for the image.
		 */
		void onScaleFactorChanged(const double dScaleFactor);

		/**
		 * Captures the indication that face features were selected or unselected in the editor.
		 */
		void onFaceFeaturesSelectionChanged();

		/**
		 * Captures indications of changes in the data model or the face widget
		 * (so the UI can be updated accordingly).
		 * @param bModified Boolean indicating if the child window shall be marked as modified or not.
		 * The default is true.
		 */
		void onDataChanged(const bool bModified = true);

		/**
		 * Captures indication of changes in the current selected image on the selection model.
		 */
		void onCurrentChanged(const QModelIndex &oCurrent, const QModelIndex &oPrevious);

	signals:

		/**
		 * Signal to indicate changes in the data model (so the UI can be updated accordingly).
		 */
		void onDataModified();

		/**
		 * Signal to indicate that the selection of face features changed in the editor.
		 * The selection can be queried through getSelectedFeatures() and getSelectedConnections().
		 */
		void onFeaturesSelectionChanged();

		/**
		 * Signal to indicate an update in the UI due to changes in the selection model.
		 * @param sImageName QString with the name of the current selected face image.
		 * @param iZoomLevel Current level of zoom in the face image widget.
		 */
		void onUIUpdated(const QString sImageName, const int iZoomLevel);

	private:

		/** Index of the current displayed face image. */
		int m_iCurrentImage;

		/** Widget used to display face images and edit facial features. */
		FaceWidget *m_pFaceWidget;

		/** Instance of the model used to encapsulate the access of the face dataset to Qt view components such as QListView. */
		FaceDatasetModel *m_pFaceDatasetModel;

		/** Selection model used to represent the selection of items in Qt view components such as QListView. */
		QItemSelectionModel *m_pFaceSelectionModel;
	};
}

#endif // CHILDWINDOW_H
