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

#include "facefeatureedge.h"
#include "facewidget.h"
#include "facefeaturenode.h"

#include <math.h>

#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

// +-----------------------------------------------------------
ft::FaceFeatureEdge::FaceFeatureEdge(FaceWidget *pFaceWidget, FaceFeatureNode *pSourceNode, FaceFeatureNode *pTargetNode)
{
	m_pFaceWidget = pFaceWidget;

    setAcceptedMouseButtons(0);

    m_pSourceNode = pSourceNode;
    m_pTargetNode = pTargetNode;

    m_pTargetNode->addEdge(this);
    m_pSourceNode->addEdge(this);

    adjust();
}

// +-----------------------------------------------------------
ft::FaceFeatureNode* ft::FaceFeatureEdge::sourceNode() const
{
    return m_pSourceNode;
}

// +-----------------------------------------------------------
ft::FaceFeatureNode* ft::FaceFeatureEdge::targetNode() const
{
    return m_pTargetNode;
}

// +-----------------------------------------------------------
void ft::FaceFeatureEdge::adjust()
{
    if (!m_pSourceNode || !m_pTargetNode)
        return;

    QLineF line(mapFromItem(m_pSourceNode, 0, 0), mapFromItem(m_pTargetNode, 0, 0));
    qreal length = line.length();

    prepareGeometryChange();

    QPointF edgeOffset((line.dx() * FaceFeatureNode::RADIUS) / length, (line.dy() * FaceFeatureNode::RADIUS) / length);
	m_oSourcePoint = line.p1() + edgeOffset;
    m_oTargetPoint = line.p2() - edgeOffset;
}

// +-----------------------------------------------------------
QRectF ft::FaceFeatureEdge::boundingRect() const
{
    if (!m_pSourceNode || !m_pTargetNode)
        return QRectF();

    QRectF oRet = QRectF(m_oSourcePoint, QSizeF(m_oTargetPoint.x() - m_oSourcePoint.x(), m_oTargetPoint.y() - m_oSourcePoint.y()));
	return oRet.normalized();
}

// +-----------------------------------------------------------
void ft::FaceFeatureEdge::paint(QPainter *pPainter, const QStyleOptionGraphicsItem *pOption, QWidget *pWidget)
{
	Q_UNUSED(pOption);
	Q_UNUSED(pWidget);
    if (!m_pSourceNode || !m_pTargetNode)
        return;

    QLineF oLine(m_oSourcePoint, m_oTargetPoint);
    pPainter->setPen(QPen(Qt::yellow, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    pPainter->drawLine(oLine);
}