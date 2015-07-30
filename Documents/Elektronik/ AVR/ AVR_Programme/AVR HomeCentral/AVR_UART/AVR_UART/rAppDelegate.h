//
//  rAppDelegate.h
//  AVR_UART
//
//  Created by Ruedi Heimlicher on 02.Februar.13.
//  Copyright (c) 2013 Ruedi Heimlicher. All rights reserved.
//

#import <Cocoa/Cocoa.h>

@interface rAppDelegate : NSObject <NSApplicationDelegate>

@property (assign) IBOutlet NSWindow *window;

@property (readonly, strong, nonatomic) NSPersistentStoreCoordinator *persistentStoreCoordinator;
@property (readonly, strong, nonatomic) NSManagedObjectModel *managedObjectModel;
@property (readonly, strong, nonatomic) NSManagedObjectContext *managedObjectContext;

- (IBAction)saveAction:(id)sender;

@end
