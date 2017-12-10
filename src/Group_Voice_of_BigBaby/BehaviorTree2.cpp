/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <behavior_tree.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BehaviorTree2");
    try
    {
        int TickPeriod_milliseconds = 50;

        BT::ROSCondition* Answer = new BT::ROSCondition("Answer");
        BT::ROSAction* TextToVoice = new BT::ROSAction("TextToVoice");
        BT::ROSAction* VoicePlay = new BT::ROSAction("VoicePlay");

        BT::ROSAction* OrderSearch = new BT::ROSAction("OrderSearch");
        BT::ROSAction* TextToText = new BT::ROSAction("TextToText");

        BT:: FallbackNodeWithMemory* Child = new BT::FallbackNodeWithMemory("Child");

        Child->AddChild(OrderSearch);
        Child->AddChild(TextToText);

        BT:: SequenceNodeWithMemory* root = new BT::SequenceNodeWithMemory("root");

        root->AddChild(Answer);
        root->AddChild(Child);
        root->AddChild(TextToVoice);
        root->AddChild(VoicePlay);

        Execute(root, TickPeriod_milliseconds);  // from BehaviorTree.cpp
    }
    catch (BT::BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}
