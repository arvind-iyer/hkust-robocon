//
//  RoboconCtrl_MacTests.m
//  RoboconCtrl-MacTests
//
//  Created by Hauton TSANG on 20/4/15.
//  Copyright (c) 2015 Hauton TSANG. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import <XCTest/XCTest.h>

@interface RoboconCtrl_MacTests : XCTestCase

@end

@implementation RoboconCtrl_MacTests

- (void)setUp {
    [super setUp];
    // Put setup code here. This method is called before the invocation of each test method in the class.
}

- (void)tearDown {
    // Put teardown code here. This method is called after the invocation of each test method in the class.
    [super tearDown];
}

- (void)testExample {
    // This is an example of a functional test case.
    XCTAssert(YES, @"Pass");
}

- (void)testPerformanceExample {
    // This is an example of a performance test case.
    [self measureBlock:^{
        // Put the code you want to measure the time of here.
    }];
}

@end
