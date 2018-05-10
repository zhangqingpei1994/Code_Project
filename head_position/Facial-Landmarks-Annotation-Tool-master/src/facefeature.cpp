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

#include "facefeature.h"
#include <QApplication>
#include <algorithm>

using namespace std;

// +-----------------------------------------------------------
ft::FaceFeature::FaceFeature():	QPoint()
{
	setID(-1);
}

// +-----------------------------------------------------------
ft::FaceFeature::FaceFeature(int iID, float x, float y): QPoint(x, y)
{
	setID(iID);
}

// +-----------------------------------------------------------
int ft::FaceFeature::getID() const
{
	return m_iID;
}

// +-----------------------------------------------------------
void ft::FaceFeature::setID(int iID)
{
	m_iID = iID;
}

// +-----------------------------------------------------------
void ft::FaceFeature::connectTo(FaceFeature *pOther)
{
	vector<int>::iterator it = find(m_vConnections.begin(), m_vConnections.end(), pOther->getID());
	if (it == m_vConnections.end())
		m_vConnections.push_back(pOther->getID());
}

// +-----------------------------------------------------------
void ft::FaceFeature::disconnectFrom(FaceFeature *pOther)
{
	vector<int>::iterator it = find(m_vConnections.begin(), m_vConnections.end(), pOther->getID());
	if (it != m_vConnections.end())
		m_vConnections.erase(it);
}

// +-----------------------------------------------------------
std::vector<int> ft::FaceFeature::getConnections()
{
	return m_vConnections;
}

// +-----------------------------------------------------------
bool ft::FaceFeature::loadFromXML(const QDomElement &oElement, QString &sMsgError)
{
// Check the element name
	if(oElement.tagName() != "Feature")
	{
		sMsgError = QString(QApplication::translate("FaceFeature", "invalid node name [%1] - expected node '%2'").arg(oElement.tagName(), "Feature"));
		return false;
	}

	QString sID = oElement.attribute("id");
	if(sID == "")
	{
		sMsgError = QString(QApplication::translate("FaceFeature", "the attribute '%1' does not exist or it contains an invalid value").arg("id"));
		return false;
	}

	QString sValueX = oElement.attribute("x");
	if(sValueX == "")
	{
		sMsgError = QString(QApplication::translate("FaceFeature", "the attribute '%1' does not exist or it contains an invalid value").arg("x"));
		return false;
	}

	QString sValueY = oElement.attribute("y");
	if(sValueY == "")
	{
		sMsgError = QString(QApplication::translate("FaceFeature", "the attribute '%1' does not exist or it contains an invalid value").arg("y"));
		return false;
	}

	// Connetions
	QDomElement oConnections = oElement.firstChildElement("Connections");
	if (oConnections.isNull())
	{
		sMsgError = QString(QApplication::translate("FaceFeature", "the node '%1' does not exist")).arg("Connections");
		return false;
	}

	vector<int> vConnections;
	for (QDomElement oConn = oConnections.firstChildElement(); !oConn.isNull(); oConn = oConn.nextSiblingElement())
	{
		QString sValue = oConn.attribute("id");
		if (sValue == "")
		{
			sMsgError = QString(QApplication::translate("FaceFeature", "the attribute '%1' does not exist or it contains an invalid value").arg("id"));
			return false;
		}

		vConnections.push_back(sValue.toInt());
	}

	m_iID = sID.toInt();
	setX(sValueX.toFloat());
	setY(sValueY.toFloat());
	m_vConnections = vConnections;
	
	return true;
}

// +-----------------------------------------------------------
void ft::FaceFeature::saveToXML(QDomElement &oParent) const
{
	QDomElement oFeature = oParent.ownerDocument().createElement("Feature");
	oParent.appendChild(oFeature);

	// Save the feature attributes
	oFeature.setAttribute("id", m_iID);
	oFeature.setAttribute("x", x());
	oFeature.setAttribute("y", y());

	// Add the "Connections" subnode
	QDomElement oConnections = oParent.ownerDocument().createElement("Connections");
	oFeature.appendChild(oConnections);

	// Save all the connections
	QDomElement oTarget;
	foreach(int iID, m_vConnections)
	{
		oTarget = oParent.ownerDocument().createElement("Target");
		oConnections.appendChild(oTarget);
		oTarget.setAttribute("id", iID);
	}
}
