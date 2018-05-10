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

#include "facewidget.h"
#include "facewidgetscene.h"

#include <QApplication>
#include <QPixmap>
#include <QGraphicsEffect>
#include <QScrollBar>
#include <QtMath>
#include <QGraphicsSceneMouseEvent>

// Scale values for zoom in and out steps
const double ft::FaceWidget::ZOOM_IN_STEP = 1.25;
const double ft::FaceWidget::ZOOM_OUT_STEP = 0.80;

// Number of face features edited by the widget
const int ft::FaceWidget::NUM_FACE_FEATURES = 68;

// +-----------------------------------------------------------
ft::FaceWidget::FaceWidget(QWidget *pParent) : QGraphicsView(pParent)
{
	setDragMode(RubberBandDrag);

	m_pScene = (QGraphicsScene*) new FaceWidgetScene(this);
	m_pScene->setItemIndexMethod(QGraphicsScene::NoIndex);
	setScene(m_pScene);
	connect(m_pScene, SIGNAL(selectionChanged()), this, SLOT(onSelectionChanged()));

	setCacheMode(CacheBackground);
    setViewportUpdateMode(BoundingRectViewportUpdate);
    setRenderHint(QPainter::Antialiasing);
    setTransformationAnchor(AnchorUnderMouse);

    scale(1.0, 1.0);
	m_dScaleFactor = 1.0;

	setBackgroundBrush(QApplication::palette().dark());

	// Add the image item
	QPixmap oPixmap(":/images/noface");
	m_pPixmapItem = m_pScene->addPixmap(oPixmap);
	m_pPixmapItem->setShapeMode(QGraphicsPixmapItem::BoundingRectShape);
	m_pScene->setSceneRect(0, 0, oPixmap.width(), oPixmap.height());

	// Setup the face features editor
	m_bDisplayFaceFeatures = true;
	m_bDisplayConnections = true;
	m_bDisplayFeatureIDs = false;

	m_pContextMenu = NULL;

	//createFaceFeatures();
	m_bFeaturesMoved = false;
	m_bSelectionChanged = false;
}

// +-----------------------------------------------------------
ft::FaceWidget::~FaceWidget()
{
	delete m_pScene;
}

// +-----------------------------------------------------------
void ft::FaceWidget::setPixmap(const QPixmap &oPixmap)
{
	m_pPixmapItem->setPixmap(oPixmap);
	m_pScene->setSceneRect(0, 0, oPixmap.width(), oPixmap.height());
}

// +-----------------------------------------------------------
double ft::FaceWidget::getScaleFactor() const
{
	return qFloor(m_dScaleFactor * 100000.0) / 100000.0; // Returns it with precision of 5 decimals
}

// +-----------------------------------------------------------
void ft::FaceWidget::setScaleFactor(const double dScaleFactor)
{
	if(dScaleFactor == m_dScaleFactor)
		return;

	if(dScaleFactor >= 0.10 && dScaleFactor <= 10.0)
	{
		// First, go back to the base scale (1.0 or 100%)
		double dAdjust = 1.0 / m_dScaleFactor;
		scale(dAdjust, dAdjust);

		// Then, apply the requested factor.
		m_dScaleFactor = dScaleFactor;
		if(m_dScaleFactor != 1.0)
			scale(dScaleFactor, dScaleFactor);
	}
}

// +-----------------------------------------------------------
void ft::FaceWidget::scaleViewBy(double dFactorBy)
{
	double dFactor = m_dScaleFactor * dFactorBy;
	if(dFactor >= 0.10 && dFactor <= 10.0)
	{
		m_dScaleFactor = dFactor;
	    scale(dFactorBy, dFactorBy);

		// Emit the signal that the scale factor has changed
		emit onScaleFactorChanged(getScaleFactor());
	}
}

#ifndef QT_NO_WHEELEVENT
// +-----------------------------------------------------------
void ft::FaceWidget::wheelEvent(QWheelEvent *pEvent)
{
	bool bCtrl = QApplication::keyboardModifiers() & Qt::ControlModifier;
	bool bAlt = QApplication::keyboardModifiers() & Qt::AltModifier;
	bool bShift = QApplication::keyboardModifiers() & Qt::ShiftModifier;
	int iDelta = pEvent->angleDelta().x() + pEvent->angleDelta().y();
	double dBase = iDelta < 0 ? ZOOM_OUT_STEP : ZOOM_IN_STEP;
	int iSteps = abs(iDelta / 120);

	if(!(bCtrl || bAlt || bShift)) // No special key pressed => scroll vertically
		verticalScrollBar()->setValue(verticalScrollBar()->value() - iDelta);
	else if(bShift && !(bCtrl || bAlt)) // Only shift key pressed => scroll horizontally
		horizontalScrollBar()->setValue(horizontalScrollBar()->value() - iDelta);
	else if(bCtrl && !(bAlt || bShift)) // Only ctrl key pressed => zoom in and out
		scaleViewBy(qPow(dBase, iSteps));
}
#endif

// +-----------------------------------------------------------
void ft::FaceWidget::mousePressEvent(QMouseEvent* pEvent)
{
	QPointF oPos = pEvent->pos();
	oPos = mapToScene(pEvent->pos());
	QTransform oTrf;
	QGraphicsItem* pItem = scene()->itemAt(oPos, oTrf);
	if (pItem && pItem->isEnabled() && (pItem->flags() & QGraphicsItem::ItemIsSelectable) &&
		(pEvent->modifiers() & Qt::ShiftModifier))
	{
		int iFirst, iLast;
		QList<FaceFeatureNode*> lsSelected = getSelectedFeatures();
		if(!lsSelected.size())
			iFirst = 0;
		else
		{
			iFirst = lsSelected.first()->getID();
			foreach(FaceFeatureNode *pNode, lsSelected)
				if(pNode->getID() < iFirst)
					iFirst = pNode->getID();
		}

		iLast = ((FaceFeatureNode*) pItem)->getID();

		if(iFirst != iLast)
		{
			QList<FaceFeatureNode*> lsFeats = getFaceFeatures();
			foreach(FaceFeatureNode* pFeat, lsFeats)
				pFeat->setSelected(false);

			for(int i = qMin(iFirst, iLast); i <= qMax(iFirst, iLast); i++)
				lsFeats[i]->setSelected(true);
		}
		pEvent->accept();
	}
	else
		QGraphicsView::mousePressEvent((QMouseEvent*) pEvent);
}

// +-----------------------------------------------------------
void ft::FaceWidget::mouseReleaseEvent(QMouseEvent *pEvent)
{
	if(m_bFeaturesMoved)
	{
		emit onFaceFeaturesChanged();
		m_bFeaturesMoved = false;
	}

	if(m_bSelectionChanged)
	{
		emit onFaceFeaturesSelectionChanged();
		m_bSelectionChanged = false;
	}

	QGraphicsView::mouseReleaseEvent(pEvent);
}

// +-----------------------------------------------------------
void ft::FaceWidget::zoomIn()
{
	scaleViewBy(ZOOM_IN_STEP);
}

// +-----------------------------------------------------------
void ft::FaceWidget::zoomOut()
{
	scaleViewBy(ZOOM_OUT_STEP);
}

// +-----------------------------------------------------------
void ft::FaceWidget::createFaceFeatures()
{
	// These are the coordinates for the face features of a sample face model
	double aFaceModel[NUM_FACE_FEATURES][2] =
	{
		{217.79878, 217.08728},
		{216.89078, 254.86311},
		{223.67206, 292.07462},
		{238.23885, 326.79248},
		{254.94789, 360.61587},
		{279.53139, 388.90414},
		{310.45043, 410.42450},
		{345.37818, 425.29204},
		{382.36266, 428.51270},
		{418.06689, 424.13911},
		{449.99561, 408.56366},
		{477.42884, 385.63278},
		{499.83194, 357.72590},
		{509.49470, 323.37268},
		{516.13174, 287.86876},
		{518.15853, 252.06194},
		{518.69634, 216.55158},
		{247.62010, 180.34162},
		{268.70792, 165.39026},
		{293.56778, 158.93545},
		{319.16937, 160.78699},
		{343.48894, 169.15112},
		{397.25624, 164.28431},
		{420.26527, 156.48688},
		{443.57508, 150.79923},
		{467.19522, 154.84270},
		{488.87233, 165.42594},
		{372.77664, 199.50671},
		{373.99586, 223.36618},
		{375.57578, 246.90136},
		{378.10595, 270.34028},
		{345.61168, 293.24677},
		{361.62731, 300.03883},
		{378.63337, 301.76439},
		{393.55789, 298.57543},
		{406.85187, 291.64665},
		{280.10863, 206.43671},
		{300.22526, 197.40182},
		{321.95062, 195.23294},
		{339.75602, 206.97070},
		{319.74604, 209.54150},
		{299.69791, 210.59487},
		{405.25688, 203.23706},
		{420.24655, 189.68711},
		{440.47753, 188.93026},
		{459.57889, 196.31247},
		{442.35281, 203.82019},
		{423.65388, 206.10059},
		{329.76923, 340.09600},
		{344.93415, 328.45414},
		{359.76855, 316.54668},
		{377.56618, 319.87236},
		{395.18127, 314.47085},
		{411.21730, 325.09745},
		{427.09559, 335.85024},
		{415.75540, 349.79882},
		{398.78054, 356.02983},
		{380.76123, 358.75053},
		{361.44337, 358.76781},
		{343.08983, 353.52445},
		{341.63086, 335.00864},
		{353.49249, 334.72611},
		{377.56554, 336.91333},
		{402.31187, 331.58481},
		{414.70373, 331.84948},
		{401.78216, 332.11416},
		{377.03251, 337.44653},
		{353.00775, 335.29117},
	};

	// Add the face feature nodes
	FaceFeatureNode *pPrevFeat = NULL;
	FaceFeatureNode *pCurFeat = NULL;
	for(int i = 0; i < NUM_FACE_FEATURES; i++)
	{
		pCurFeat = addFaceFeature(QPoint(aFaceModel[i][0], aFaceModel[i][1]));
		if(!pPrevFeat)
			pPrevFeat = pCurFeat;
		else
		{
			connectFaceFeatures(pPrevFeat, pCurFeat);
			pPrevFeat = pCurFeat;
		}
	}
}

// +-----------------------------------------------------------
const QList<ft::FaceFeatureNode*>& ft::FaceWidget::getFaceFeatures(int iNumFeats)
{
	if(iNumFeats != -1)
	{
		int iDiff = m_lFaceFeatures.size() - iNumFeats;

		if(iDiff > 0)
		{
			while(iDiff-- > 0)
				removeFaceFeature(m_lFaceFeatures.last());
		}
		else if(iDiff < 0)
		{
			while(iDiff++ < 0)
				addFaceFeature();
		}

	}
	return m_lFaceFeatures;
}

// +-----------------------------------------------------------
QList<ft::FaceFeatureNode*> ft::FaceWidget::getSelectedFeatures() const
{
	QList<FaceFeatureNode*> lSelected;
	foreach(QGraphicsItem *pItem, m_pScene->selectedItems())
		lSelected.append((FaceFeatureNode*) pItem);
	return lSelected;
}

// +-----------------------------------------------------------
QList<ft::FaceFeatureEdge*> ft::FaceWidget::getSelectedConnections() const
{
	QList<FaceFeatureNode*> lFeatures = getSelectedFeatures();
	QList<FaceFeatureEdge*> lSelected;

	QList<FaceFeatureNode*>::iterator oFirst, oSecond;
	FaceFeatureEdge *pEdge;

	for(oFirst = lFeatures.begin(); oFirst != lFeatures.end(); ++oFirst)
	{
		for(oSecond = oFirst + 1; oSecond != lFeatures.end(); ++oSecond)
		{
			pEdge = (*oFirst)->getEdgeTo(*oSecond);
			if(pEdge)
			{
				lSelected.append(pEdge);
				break;
			}
		}
	}

	return lSelected;
}

// +-----------------------------------------------------------
ft::FaceFeatureNode* ft::FaceWidget::addFaceFeature(const QPoint &oPos, bool bGlobal)
{
	int iID = m_lFaceFeatures.size();
	FaceFeatureNode *pNode = new FaceFeatureNode(iID, this);
	m_pScene->addItem(pNode);
	m_lFaceFeatures.append(pNode);
	if(bGlobal)
		pNode->setPos(mapToScene(mapFromGlobal(oPos)));
	else
		pNode->setPos(oPos);
	return pNode;
}

// +-----------------------------------------------------------
void ft::FaceWidget::removeFaceFeature(FaceFeatureNode* pNode)
{
	// First, remove all edges connected to the node
	QList<FaceFeatureEdge*> lEdges = pNode->edges();
	for(int i = 0; i < lEdges.size(); i++)
		removeConnection(lEdges[i]);

	// Then, remove the node
	m_lFaceFeatures.removeOne(pNode);
	m_pScene->removeItem(pNode);
	delete pNode;

	// And adjust the IDs of the remaining features
	int iID = 0;
	foreach(FaceFeatureNode *pNode, m_lFaceFeatures)
		pNode->setID(iID++);
}

// +-----------------------------------------------------------
ft::FaceFeatureEdge* ft::FaceWidget::connectFaceFeatures(FaceFeatureNode* pSource, FaceFeatureNode* pTarget)
{
	FaceFeatureEdge* pEdge = pSource->getEdgeTo(pTarget);
	if(pEdge)
		return pEdge;

	pEdge = new FaceFeatureEdge(this, pSource, pTarget);
	m_pScene->addItem(pEdge);
	m_lConnections.append(pEdge);
	return pEdge;
}

// +-----------------------------------------------------------
ft::FaceFeatureEdge* ft::FaceWidget::connectFaceFeatures(int iSource, int iTarget)
{
	FaceFeatureNode *pSource = NULL, *pTarget = NULL;
	QList<FaceFeatureNode *> lFeats = getFaceFeatures();
	foreach(FaceFeatureNode *pFeat, lFeats)
	{
		if (pFeat->getID() == iSource)
			pSource = pFeat;
		else if (pFeat->getID() == iTarget)
			pTarget = pFeat;

		if (pSource && pTarget)
			break;
	}

	if (pSource && pTarget)
		return connectFaceFeatures(pSource, pTarget);
	else
		return NULL;
}

// +-----------------------------------------------------------
void ft::FaceWidget::disconnectFaceFeatures(FaceFeatureNode* pSource, FaceFeatureNode* pTarget)
{
	FaceFeatureEdge *pEdge = pSource->getEdgeTo(pTarget);
	if(pEdge)
		removeConnection(pEdge);
}

// +-----------------------------------------------------------
void ft::FaceWidget::removeConnection(FaceFeatureEdge* pEdge)
{
	pEdge->sourceNode()->removeEdge(pEdge);
	pEdge->targetNode()->removeEdge(pEdge);
	m_lConnections.removeOne(pEdge);
	m_pScene->removeItem(pEdge);
	delete pEdge;
}

// +-----------------------------------------------------------
void ft::FaceWidget::faceFeatureMoved(FaceFeatureNode *pNode)
{
	Q_UNUSED(pNode);
	m_bFeaturesMoved = true;
}

// +-----------------------------------------------------------
void ft::FaceWidget::onSelectionChanged()
{
	m_bSelectionChanged = true;
}

// +-----------------------------------------------------------
bool ft::FaceWidget::displayFaceFeatures() const
{
	return m_bDisplayFaceFeatures;
}
// +-----------------------------------------------------------
void ft::FaceWidget::setDisplayFaceFeatures(const bool bValue)
{
	if(bValue == m_bDisplayFaceFeatures)
		return;

	m_bDisplayFaceFeatures = bValue;
	foreach(FaceFeatureNode *pNode, m_lFaceFeatures)
		pNode->setVisible(bValue);
	update();
}

// +-----------------------------------------------------------
bool ft::FaceWidget::displayConnections() const
{
	return m_bDisplayConnections;
}

// +-----------------------------------------------------------
void ft::FaceWidget::setDisplayConnections(const bool bValue)
{
	if(bValue == m_bDisplayConnections)
		return;

	m_bDisplayConnections = bValue;
	foreach(FaceFeatureEdge *pEdge, m_lConnections)
		pEdge->setVisible(bValue);
	update();
}

// +-----------------------------------------------------------
bool ft::FaceWidget::displayFeatureIDs() const
{
	return m_bDisplayFeatureIDs;
}

// +-----------------------------------------------------------
void ft::FaceWidget::setDisplayFeatureIDs(const bool bValue)
{
	m_bDisplayFeatureIDs = bValue;
	foreach(FaceFeatureNode *pNode, m_lFaceFeatures)
		pNode->update();
	update();
}

// +-----------------------------------------------------------
void ft::FaceWidget::contextMenuEvent(QContextMenuEvent *pEvent)
{
	if(m_pContextMenu)
	{
		// Set the context menu mouse position as the user data of the
		// first action (the 'add feature' action), so it can be used
		// as the position of the new feature point
		m_pContextMenu->actions()[0]->setData(pEvent->globalPos());

		// call the context menu
		m_pContextMenu->exec(pEvent->globalPos());

		// After the menu is closed, empty the data to avoid using it wrongly in the future
		m_pContextMenu->actions()[0]->setData(QVariant::Invalid);
	}
}

// +-----------------------------------------------------------
void ft::FaceWidget::setContextMenu(QMenu *pMenu)
{
	m_pContextMenu = pMenu;
}