/*
 * Copyright (C) 2023 eSOL Co.,Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE FREEBSD PROJECT ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE FREEBSD PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <QApplication>

// include layout
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

// include object
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);

  QWidget * window1 = new QWidget;
  QPushButton * button1A = new QPushButton("Button 1A");
  QPushButton * button1B = new QPushButton("Button 1B");
  QPushButton * button1C = new QPushButton("Button 1C");
  QHBoxLayout * layout1 = new QHBoxLayout;
  layout1->addWidget(button1A);
  layout1->addWidget(button1B);
  layout1->addWidget(button1C);
  window1->setLayout(layout1);
  window1->show();

  QWidget * window2 = new QWidget;
  QPushButton * button2A = new QPushButton("Button 2A");
  QPushButton * button2B = new QPushButton("Button 2B");
  QPushButton * button2C = new QPushButton("Button 2C");
  QGridLayout * layout2 = new QGridLayout;
  layout2->addWidget(button2A, 0, 0);
  layout2->addWidget(button2B, 0, 1);
  layout2->addWidget(button2C, 1, 0, 1, 2);
  window2->setLayout(layout2);
  window2->show();

  QWidget * window3 = new QWidget;
  QLabel * label = new QLabel("###Label###");
  QLineEdit * edit = new QLineEdit("###LineEdit###");
  QVBoxLayout * layout3 = new QVBoxLayout;
  layout3->addWidget(label);
  layout3->addWidget(edit);
  window3->setLayout(layout3);
  window3->show();

  return app.exec();
}
