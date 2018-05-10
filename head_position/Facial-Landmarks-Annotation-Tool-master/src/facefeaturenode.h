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

#ifndef FACEFEATURENODE_H
#define FACEFEATURENODE_H

#include <QGraphicsItem>
#include <QList>

namespace ft
{
	class FaceWidget;
	class FaceFeatureEdge;

	/**
	 * Implements the visual manipulator of facial feature nodes to be used together with the FaceWidget.
	 */
	class FaceFeatureNode: public QGraphicsItem
	{
	public:
		/**
		 * Class constructor.
		 * @param iID Integer with the ID of the feature node.
		 * @param pFaceWidget Instance of the FaceWidget to be the parent of the face feature node.
		 */
		FaceFeatureNode(int iID, FaceWidget *pFaceWidget);

		/**
		 * Adds a new edge to the face feature node.
		 * @param pEdge Instance of the FaceFeatureEdge to be added to the face feature.
		 */
		void addEdge(FaceFeatureEdge *pEdge);

		/**
		 * Removes an existing edge from the face feature node.
		 * @param pEdge Instance of the FaceFeatureEdge to be removed from the face feature.
		 */
		void removeEdge(FaceFeatureEdge *pEdge);

		/**
		 * Queries the list of face feature edges connected to this node.
		 * @return Q QList<FaceFeatureEdge*> with the list of edges connected to this node.
		 */
		QList<FaceFeatureEdge *> edges() const;

		/**
		 * Queries the edge that connects this node to the given node.
		 * @param pNode Instance of the other FaceFeatureNode that might be connected to this node.
		 * @return An instance of the FaceFeatureEdge with the edge connecting the two nodes, or NULL
		 * if there is no edge connecting the nodes.
		 */
		FaceFeatureEdge *getEdgeTo(const FaceFeatureNode *pNode) const;

		/**
		 * Queries the bounding rectangle of the face feature node.
		 * @return A QRectF with the coordinates and size of the bounding rect of the node.
		 */
		QRectF boundingRect() const Q_DECL_OVERRIDE;

		/**
		 * Gets the identifier of the feature node.
		 */
		int getID() const;

		/**
		 * Updates the identifier of the feature node.
		 * @param iID Integer with the new identifier for the feature node.
		 */
		void setID(int iID);

	public:

		/** Constant with the radius of the node drawn, in pixels. */
		const static int RADIUS;

	protected:

		/**
		 * Paint method for the face feature node.
		 * @param pPainter Instance of a QPainter to allow drawing the node.
		 * @param pOption Instance of a QStyleOptionGraphicsItem with information on the style and state of the node.
		 * @param pWidget Instance of a QWidget with the widget that the node is being painted on. Optional, and might be 0.
		 */
		void paint(QPainter *pPainter, const QStyleOptionGraphicsItem *pOption, QWidget *pWidget) Q_DECL_OVERRIDE;

		/**
		 * Captures the event of changes in the node.
		 * @param eChange GraphicsItemChange enumeration value indicating what is the change.
		 * @param oValue QVariant const reference with the value that changed.
		 */
		QVariant itemChange(GraphicsItemChange eChange, const QVariant &oValue) Q_DECL_OVERRIDE;

		/**
		 * Captures the mouse enter event on the feature node.
		 * @param pEvent Instance of a QGraphicsSceneHoverEvent with the event data.
		 */
		void hoverEnterEvent(QGraphicsSceneHoverEvent *pEvent) Q_DECL_OVERRIDE;

		/**
		 * Captures the mouse exit event on the feature node.
		 * @param pEvent Instance of a QGraphicsSceneHoverEvent with the event data.
		 */
		void hoverLeaveEvent(QGraphicsSceneHoverEvent *pEvent) Q_DECL_OVERRIDE;

	private:

		/** Reference to the parent face widget. */
		FaceWidget *m_pFaceWidget;

		/** List of edges added to the node. */
		QList<FaceFeatureEdge *> m_lEdges;

		/** Identifier of the face feature node. */
		int m_iID;
	};
};

#endif // FACEFEATURENODE_H
