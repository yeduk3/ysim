#import <Cocoa/Cocoa.h>

void setDockIcon(const char* path) {
    @autoreleasepool {
        NSString* nsPath = [NSString stringWithUTF8String:path];
        NSImage* img = [[NSImage alloc] initWithContentsOfFile:nsPath];
        if (img) {
            [NSApp setApplicationIconImage:img];
        }
    }
}
