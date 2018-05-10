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

#ifndef FACEFEATUREEDGE_H
#define FACEFEATUREEDGE_H

#include <QGraphicsItem>

namespace ft
{
	class FaceWidget;
	class FaceFeatureNode;

	/**
	 * Implements the edges connecting two FaceFeatures.
	 */ 
	class FaceFeatureEdge : public QGraphicsItem
	{
	public:
		/**
		 * Class constructor.
		 * @param pFaceWidget Instance of the FaceWidget to be the parent of the face feature edge.
		 * @param pSourceFeat Instance of the FaceFeatureNode that acts as the first point of the edge.
		 * @param pTargetNode Instance of the FaceFeatureNode that acts as the second point of the edge.
		 */
		FaceFeatureEdge(FaceWidget *pFaceWidget, FaceFeatureNode *pSourceNode, FaceFeatureNode *pTargetNode);

		/**
		 * Gets the first point of the edge.
		 * @return An instance of FaceFeatureNode with the first point of the edge.
		 */
		FaceFeatureNode *sourceNode() const;

		/**
		 * Gets the second point of the edge.
		 * @return An instance of FaceFeatureNode with the second point of the edge.
		 */
		FaceFeatureNode *targetNode() const;

		/**
		 * Forces the edge to adjust its coordinates based on the first (source) and second (target) face feature nodes.
		 */
		void adjust();

	protected:

		/**
		 * Queries the bounding rectangle of the face feature edge.
		 * @return A QRectF with the coordinates and size of the bounding rect of the edge.
		 */
		QRectF boundingRect() const Q_DECL_OVERRIDE;

		/**
		 * Paint method for the face feature edge.
		 * @param pPainter Instance of a QPainter to allow drawing the edge.
		 * @param pOption Instance of a QStyleOptionGraphicsItem with information on the style and state of the edge.
		 * @param pWidget Instance of a QWidget with the widget that the edge is being painted on. Optional, and might be 0.
		 */
		void paint(QPainter *pPainter, const QStyleOptionGraphicsItem *pOption, QWidget *pWidget) Q_DECL_OVERRIDE;

	private:

		/** Reference to the parent face widget. */
		FaceWidget *m_pFaceWidget;

		/** The first (source) face feature node of the edge. */
		FaceFeatureNode *m_pSourceNode;

		/** The second (target) face feature node of the edge. */
		FaceFeatureNode *m_pTargetNode;

		/** Starting point for the face feature edge. */
		QPointF m_oSourcePoint;

		/** Ending point for the face feature edge. */
		QPointF m_oTargetPoint;
	};
};

#endif // FACEFEATUREEDGE_H
