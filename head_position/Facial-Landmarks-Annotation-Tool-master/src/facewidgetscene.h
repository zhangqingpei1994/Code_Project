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

#ifndef FACEWIDGETSCENE_H
#define FACEWIDGETSCENE_H

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

namespace ft
{
	/**
	 * Inherits the base QGraphicsScene to implement specific interaction behaviours in the face features editor.
	 */
	class FaceWidgetScene: QGraphicsScene
	{
	public:
		/**
		 * Class constructor.
		 * @param pParent Instance of a QObject with the parent for the scene.
		 */
		FaceWidgetScene(QObject *pParent = 0);

	protected:

		/**
		 * Captures the mouse press event.
		 * @param pEvent Instance of a QGraphicsSceneMouseEvent with the event data.
		 */
		void mousePressEvent(QGraphicsSceneMouseEvent *pEvent) Q_DECL_OVERRIDE;
	};
};

#endif //FACEWIDGETSCENE_H