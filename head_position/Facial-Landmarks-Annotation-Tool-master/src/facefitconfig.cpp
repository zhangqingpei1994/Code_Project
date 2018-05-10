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

#include "facefitconfig.h"
#include "ui_facefitconfig.h"
#include <QFileDialog>
#include <QPushButton>
#include <QFileInfo>
#include <QMessageBox>

// +-----------------------------------------------------------
ft::FaceFitConfig::FaceFitConfig(QWidget *pParent) :
	QDialog(pParent, Qt::WindowSystemMenuHint | Qt::WindowTitleHint),
	ui(new Ui::FaceFitConfig)
{
    ui->setupUi(this);
	setFixedSize(size());
	connect(ui->pathButton, SIGNAL(clicked()), this, SLOT(on_actionSelectPath_triggered()));
	connect(ui->pathEdit, SIGNAL(textChanged(QString)), this, SLOT(onPathChanged(QString)));
	connect(ui->buttonBox, SIGNAL(helpRequested()), this, SLOT(onShowHelp()));
	onPathChanged(ui->pathEdit->text());
}

// +-----------------------------------------------------------
ft::FaceFitConfig::~FaceFitConfig()
{
    delete ui;
}

// +-----------------------------------------------------------
void ft::FaceFitConfig::onPathChanged(QString sFilePath)
{
	bool bEnabled = sFilePath.length() > 0 && QFileInfo(sFilePath).exists();
	ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(bEnabled);
}

// +-----------------------------------------------------------
void ft::FaceFitConfig::onShowHelp()
{
	QMessageBox::information(this, tr("Help on the face-fit utility"), tr("The <code>face-fit</code> utility is an external executable that can be used to automatically adjust facial landmarks to an image of a human face. It is part of the <a href='http://face.ci2cv.net/'>CSIRO Face Analysis SDK</a>."), QMessageBox::Ok);
}

// +-----------------------------------------------------------
void ft::FaceFitConfig::on_actionSelectPath_triggered()
{
	QString sFile = QFileDialog::getOpenFileName(this, tr("Select face-fit executable..."), NULL, tr("All files (*.*)"));
	if (sFile.length())
		ui->pathEdit->setText(sFile);
}

// +-----------------------------------------------------------
QString ft::FaceFitConfig::getFaceFitPath()
{
	return ui->pathEdit->text();
}