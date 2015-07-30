//
//  NSView.m
//  
//
//  Created by Ruedi Heimlicher on 21.November.12.
//
//

#import "NSView.h"

@interface NSView ()

@end

@implementation NSView

- (id)initWithWindow:(NSWindow *)window
{
    self = [super initWithWindow:window];
    if (self) {
        // Initialization code here.
    }
    
    return self;
}

- (void)windowDidLoad
{
    [super windowDidLoad];
    
    // Implement this method to handle any initialization after your window controller's window has been loaded from its nib file.
}

@end
