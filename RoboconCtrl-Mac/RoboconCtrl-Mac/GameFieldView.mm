//
//  GameFieldView.m
//  RoboconCtrl-Mac
//
//  Created by Hauton TSANG on 20/4/15.
//  Copyright (c) 2015 Hauton TSANG. All rights reserved.
//

#import "GameFieldView.h"
#include <OpenGL/gl.h>

@implementation GameFieldView

static void drawAnObject()
{
    glColor3f(1.0f, 0.85f, 0.35f);
    glBegin(GL_TRIANGLES);
    {
        glVertex3f(0.0, 0.6, 0.0);
        glVertex3f(-0.2, -0.3, 0.0);
        glVertex3f(0.2, -0.3, 0.0);
    }
    glEnd();
}

-(void) drawRect: (NSRect) bounds
{
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawAnObject();
    glFlush();
}

@end
