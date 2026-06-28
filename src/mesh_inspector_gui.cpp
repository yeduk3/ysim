#include "MeshInspectorWindow.hpp"
#include "imgui.h"
#include "imgui_internal.h"
#include <cmath>
#include <cstdio>
#include <unordered_map>

namespace mesh_inspector {

static const ImVec4 kG100(0.063f,0.078f,0.102f,1), kG90(0.098f,0.122f,0.157f,1),
    kG60(0.420f,0.463f,0.518f,1), kG50(0.545f,0.584f,0.631f,1),
    kG40(0.690f,0.722f,0.757f,1), kG20(0.886f,0.906f,0.922f,1),
    kG10(0.933f,0.941f,0.949f,1), kG5(0.976f,0.980f,0.984f,1), kW(1,1,1,1);
static const float kP=24;

static int clothBI(bool g){return g?2:1;}
static int simp(int c){if(c==1||c==2)return 0;if(c==3)return 1;return -1;}

// Content width = window - padding*2
static float CW() { return ImGui::GetWindowSize().x - kP*2; }

// ─── InputXYZ ───────────────────────────────────────────────────────
static bool InputXYZ(const char* id, float v[3]) {
    bool ch=false; ImGui::PushID(id);
    float w=CW(), gap=4, chW=(w-gap*2)/3;
    const char* L[]={"x","y","z"};
    for(int i=0;i<3;++i){
        if(i)ImGui::SameLine(0,gap);
        ImGui::SetNextItemWidth(chW);
        char f[16]; std::snprintf(f,16,"%s  %%.3f",L[i]);
        ImGui::PushID(i); if(ImGui::DragFloat("##v",&v[i],0.01f,0,0,f))ch=true; ImGui::PopID();
    }
    ImGui::PopID(); return ch;
}

// ─── InputRGB ───────────────────────────────────────────────────────
static bool InputRGB(const char* id, float col[3]) {
    bool ch=false; ImGui::PushID(id);
    float w=CW(), gap=4, sw=40, chW=(w-sw-gap*3)/3;
    const char* L[]={"R","G","B"};
    for(int i=0;i<3;++i){
        if(i)ImGui::SameLine(0,gap);
        ImGui::SetNextItemWidth(chW);
        char f[16]; std::snprintf(f,16,"%s  %%.0f",L[i]);
        ImGui::PushID(i); float v=col[i]*255;
        if(ImGui::DragFloat("##c",&v,1,0,255,f)){col[i]=v/255;ch=true;} ImGui::PopID();
    }
    ImGui::SameLine(0,gap);
    ImVec4 pv(col[0],col[1],col[2],1);
    if(ImGui::ColorButton("##sw",pv,ImGuiColorEditFlags_NoTooltip,{sw,sw}))ImGui::OpenPopup("##pk");
    if(ImGui::BeginPopup("##pk")){
        if(ImGui::ColorPicker3("##p",col,ImGuiColorEditFlags_NoSidePreview|ImGuiColorEditFlags_NoSmallPreview|ImGuiColorEditFlags_NoInputs))ch=true;
        ImGui::EndPopup();
    }
    ImGui::PopID(); return ch;
}

// ─── PillToggle ─────────────────────────────────────────────────────
static bool PillToggle(const char* id, bool* v) {
    ImGui::PushID(id); bool ch=false;
    float w=42,h=24,r=h/2;
    ImVec2 pos=ImGui::GetCursorScreenPos();
    ImDrawList* dl=ImGui::GetWindowDrawList();
    if(ImGui::InvisibleButton("##t",{w,h})){*v=!*v;ch=true;}
    dl->AddRectFilled(pos,{pos.x+w,pos.y+h},*v?ImGui::ColorConvertFloat4ToU32(kG100):ImGui::ColorConvertFloat4ToU32(kG20),r);
    float tr=10,tx=*v?pos.x+w-2-tr:pos.x+2+tr;
    dl->AddCircleFilled({tx,pos.y+h/2},tr,IM_COL32(255,255,255,255),16);
    ImGui::PopID(); return ch;
}

// ─── RangeSliderInt2 ────────────────────────────────────────────────
// One track, two handles: the [*lo,*hi] span is filled (accent), the rest is a
// gray background. Drag either handle (nearest to the press point); they can't
// cross. Returns true the frame a handle moves. Integer domain [vmin,vmax].
static bool RangeSliderInt2(const char* id, int* lo, int* hi, int vmin, int vmax) {
    ImGui::PushID(id);
    bool ch=false;
    const float w=CW(), h=20, r=h*0.5f;
    ImVec2 p=ImGui::GetCursorScreenPos();
    ImDrawList* dl=ImGui::GetWindowDrawList();
    ImGui::InvisibleButton("##rs",{w,h});
    const bool active=ImGui::IsItemActive();
    const float xL=p.x+r, xR=p.x+w-r, yc=p.y+h*0.5f;
    auto v2x=[&](int v){ float t=vmax>vmin?float(v-vmin)/float(vmax-vmin):0.f; return xL+t*(xR-xL); };
    auto x2v=[&](float x){ float t=(xR>xL)?(x-xL)/(xR-xL):0.f; t=t<0?0:(t>1?1:t); return vmin+int(t*float(vmax-vmin)+0.5f); };
    ImGuiStorage* ss=ImGui::GetStateStorage();
    ImGuiID wk=ImGui::GetID("##which");
    if(ImGui::IsItemActivated()){  // pick the handle nearest the press
        float mx=ImGui::GetIO().MousePos.x;
        ss->SetInt(wk, std::fabs(mx-v2x(*lo))<=std::fabs(mx-v2x(*hi))?0:1);
    }
    if(active){
        int which=ss->GetInt(wk,0);
        int v=x2v(ImGui::GetIO().MousePos.x);
        if(which==0){ if(v>*hi)v=*hi; if(v!=*lo){*lo=v;ch=true;} }
        else        { if(v<*lo)v=*lo; if(v!=*hi){*hi=v;ch=true;} }
    }
    const ImU32 cTrack=ImGui::ColorConvertFloat4ToU32(kG20);
    const ImU32 cFill=ImGui::ColorConvertFloat4ToU32(kG100);
    dl->AddRectFilled({p.x,yc-3},{p.x+w,yc+3},cTrack,3.f);
    const float xa=v2x(*lo), xb=v2x(*hi);
    dl->AddRectFilled({xa,yc-3},{xb,yc+3},cFill,3.f);
    dl->AddCircleFilled({xa,yc},r*0.85f,IM_COL32(255,255,255,255),16);
    dl->AddCircleFilled({xb,yc},r*0.85f,IM_COL32(255,255,255,255),16);
    ImGui::PopID();
    return ch;
}

// ─── AccordionHeader ────────────────────────────────────────────────
static std::unordered_map<ImGuiID,bool> sA;
static bool AccordionHeader(const char* kr, const char* en) {
    ImGui::PushID(kr);
    ImGuiID aid=ImGui::GetID("##a");
    if(sA.find(aid)==sA.end())sA[aid]=true;
    float fW=ImGui::GetWindowSize().x, hH=56;
    ImVec2 pos=ImGui::GetCursorScreenPos();
    ImDrawList* dl=ImGui::GetWindowDrawList();
    dl->AddRectFilled(pos,{pos.x+fW,pos.y+hH},ImGui::ColorConvertFloat4ToU32(kG5));
    bool cl=ImGui::InvisibleButton("##ab",{fW,hH});
    if(cl)sA[aid]=!sA[aid];
    bool op=sA[aid];
    ImFont* fo=ImGui::GetFont();
    float tFS=ImGui::GetFontSize(), sFS=ImGui::GetFontSize()*0.85f;
    float tY=pos.y+(hH-tFS)/2, sY=pos.y+(hH-sFS)/2;
    dl->AddText(fo,tFS,{pos.x+kP,tY},ImGui::ColorConvertFloat4ToU32(kG100),kr);
    if(en&&en[0]){ImVec2 ts=fo->CalcTextSizeA(tFS,FLT_MAX,0,kr);dl->AddText(fo,sFS,{pos.x+kP+ts.x+4,sY},ImGui::ColorConvertFloat4ToU32(kG60),en);}
    float aX=pos.x+fW-kP,aY=pos.y+hH/2;float r=5;ImU32 aC=ImGui::ColorConvertFloat4ToU32(kG60);
    if(op){dl->PathLineTo({aX-r,aY+r*.4f});dl->PathLineTo({aX,aY-r*.4f});dl->PathLineTo({aX+r,aY+r*.4f});}
    else{dl->PathLineTo({aX-r,aY-r*.4f});dl->PathLineTo({aX,aY+r*.4f});dl->PathLineTo({aX+r,aY-r*.4f});}
    dl->PathStroke(aC,0,2);
    ImGui::PopID(); return op;
}

// ─── InlineSlider: fixed 60px label offset ──────────────────────────
static bool InlineSlider(const char* label, float* v, float vmin, float vmax) {
    ImGui::PushID(label);
    float contentW=CW(), labelOff=60, valW=36, gap=8;
    float sliderW=contentW-labelOff-gap-valW;
    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted(label);ImGui::PopStyleColor();
    ImGui::SameLine(kP+labelOff);
    ImGui::PushStyleColor(ImGuiCol_FrameBg,kG10);ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,kG10);ImGui::PushStyleColor(ImGuiCol_FrameBgActive,kG10);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,ImVec2(0,4));
    ImGui::SetNextItemWidth(sliderW);
    bool ch=ImGui::SliderFloat("##sl",v,vmin,vmax,"");
    ImGui::PopStyleVar();ImGui::PopStyleColor(3);
    ImGui::SameLine(kP+contentW-valW);
    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::Text("%.3f",*v);ImGui::PopStyleColor();
    ImGui::PopID();return ch;
}

// ─── SegmentTab ─────────────────────────────────────────────────────
static void SegmentTab(const char* id, const char* lA, const char* lB, int cur, int* out) {
    ImGui::PushID(id);
    ImVec2 cP=ImGui::GetCursorScreenPos();
    float cW=124,cH=40,pad=4,segH=32,segW=(cW-pad*3)/2;
    ImGui::GetWindowDrawList()->AddRectFilled(cP,{cP.x+cW,cP.y+cH},ImGui::ColorConvertFloat4ToU32(kG10),8);
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding,8);ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize,0);
    float textH=ImGui::GetFontSize();float fpy=(segH-textH)*0.5f;
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,{0,fpy});
    auto seg=[&](const char* l,int idx,float x){
        bool act=(idx==cur);
        ImGui::SetCursorScreenPos({x,cP.y+pad});
        ImGui::PushStyleColor(ImGuiCol_Button,act?kW:ImVec4(0,0,0,0));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,act?kW:ImVec4(1,1,1,.5f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,kW);
        ImGui::PushStyleColor(ImGuiCol_Text,act?kG90:kG50);
        if(ImGui::Button(l,{segW,segH}))*out=idx;
        ImGui::PopStyleColor(4);
    };
    seg(lA,0,cP.x+pad);seg(lB,1,cP.x+pad+segW+pad);
    ImGui::PopStyleVar(3);
    // DON'T move cursor — caller manages positioning
    ImGui::PopID();
}

// ─── Card ───────────────────────────────────────────────────────────
static bool CardButton(const char* id, const char* title, void(*ico)(ImDrawList*,float,float,float)) {
    ImGui::PushID(id);
    float w=CW(),h=56;
    ImVec2 cP=ImGui::GetCursorScreenPos();
    ImDrawList* dl=ImGui::GetWindowDrawList();
    bool cl=ImGui::InvisibleButton("##c",{w,h});bool hv=ImGui::IsItemHovered();
    dl->AddRectFilled(cP,{cP.x+w,cP.y+h},ImGui::ColorConvertFloat4ToU32(hv?kG10:kG5),8);
    if(ico)ico(dl,cP.x+16,cP.y+(h-36)/2,36);
    ImFont* f=ImGui::GetFont();float fs=ImGui::GetFontSize();
    dl->AddText(f,fs,{cP.x+16+36+12,cP.y+(h-fs)/2},ImGui::ColorConvertFloat4ToU32(kG100),title);
    ImGui::PopID();return cl;
}
static void IcoPlus(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2;ImU32 co=ImGui::ColorConvertFloat4ToU32(kG40);d->AddLine({c-8,m},{c+8,m},co,1.5f);d->AddLine({c,m-8},{c,m+8},co,1.5f);}
static void IcoCube(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2,z=10;d->AddQuadFilled({c,m-z},{c+z,m-z/2},{c,m},{c-z,m-z/2},IM_COL32(224,229,236,255));d->AddQuadFilled({c-z,m-z/2},{c,m},{c,m+z},{c-z,m+z/2},IM_COL32(195,202,210,255));d->AddQuadFilled({c,m},{c+z,m-z/2},{c+z,m+z/2},{c,m+z},IM_COL32(210,217,224,255));}
static void IcoSphere(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2;d->AddCircleFilled({c,m},11,IM_COL32(220,226,233,255),24);d->AddCircleFilled({c-3,m-3},5,IM_COL32(240,243,247,255),12);}
static void IcoCyl(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2;d->AddRectFilled({c-8,m-9},{c+8,m+9},IM_COL32(220,226,233,255));d->AddEllipseFilled({c,m-9},{8,4},IM_COL32(210,217,224,255),0,16);d->AddEllipseFilled({c,m+9},{8,4},IM_COL32(210,217,224,255),0,16);}
static void IcoPlane(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2;ImVec2 p[4]={{c,m-8},{c+14,m},{c,m+8},{c-14,m}};d->AddConvexPolyFilled(p,4,IM_COL32(215,222,229,255));}
static void IcoSkeleton(ImDrawList* d,float x,float y,float s){float c=x+s/2,m=y+s/2;ImU32 co=IM_COL32(195,202,210,255);d->AddCircleFilled({c,m-9},4,co,12);d->AddLine({c,m-5},{c,m+2},co,2);d->AddLine({c,m-3},{c-6,m+1},co,2);d->AddLine({c,m-3},{c+6,m+1},co,2);d->AddLine({c,m+2},{c-4,m+9},co,2);d->AddLine({c,m+2},{c+4,m+9},co,2);}

// DrawTrash icon helper
static void DrawTrash(ImDrawList* dl, float cx, float cy) {
    ImU32 ic=ImGui::ColorConvertFloat4ToU32(kG60);
    dl->AddRect({cx-6,cy-4},{cx+6,cy+8},ic,2,0,1.5f);
    dl->AddLine({cx-8,cy-4},{cx+8,cy-4},ic,1.5f);
    dl->AddLine({cx-3,cy-4},{cx-3,cy-7},ic,1.5f);dl->AddLine({cx-3,cy-7},{cx+3,cy-7},ic,1.5f);dl->AddLine({cx+3,cy-7},{cx+3,cy-4},ic,1.5f);
    dl->AddLine({cx-2,cy-1},{cx-2,cy+5},ic,1);dl->AddLine({cx,cy-1},{cx,cy+5},ic,1);dl->AddLine({cx+2,cy-1},{cx+2,cy+5},ic,1);
}

// ═════════════════════════════════════════════════════════════════════
void drawMeshInspectorWindow(MeshInspectorWindowState& st, const MeshInspectorTarget& t) {
    if(!st.open)return;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,0);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize,0);
    if(!ImGui::Begin("물체",nullptr,ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoSavedSettings|ImGuiWindowFlags_NoTitleBar)){
        ImGui::PopStyleVar(2);ImGui::End();return;}

    float winW = ImGui::GetWindowSize().x;

    // ═══ Point panel ═════════════════════════════════════════════════
    if(t.point_panel){
        ImGui::Dummy({0,kP});ImGui::Indent(kP);
        ImGui::Text("점 선택: 물체 %d / 정점 %d",t.point_obj,t.point_vert);
        ImGui::Unindent(kP);ImGui::Separator();ImGui::Dummy({0,kP});ImGui::Indent(kP);
        bool fx=t.point_fixed;if(ImGui::Checkbox("점 고정",&fx)&&t.on_point_set_fixed)t.on_point_set_fixed(fx);
        ImGui::Dummy({0,12});
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("위치");ImGui::PopStyleColor();ImGui::Dummy({0,4});
        float p[3]={t.point_position[0],t.point_position[1],t.point_position[2]};
        if(InputXYZ("pp",p)&&t.on_point_move)t.on_point_move(p[0],p[1],p[2]);
        ImGui::Dummy({0,12});
        {bool ra=t.point_ref_active;if(ra){ImGui::PushStyleColor(ImGuiCol_Button,kG90);ImGui::PushStyleColor(ImGuiCol_Text,kW);}
        if(ImGui::Button(ra?"참조점 선택 중...":"다른 점 위치 참조",{CW(),40})&&t.on_point_ref_toggle)t.on_point_ref_toggle();
        if(ra)ImGui::PopStyleColor(2);
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextWrapped("다른 점을 클릭하면 이 점이 따라갑니다.");ImGui::PopStyleColor();}
        ImGui::Unindent(kP);ImGui::Separator();ImGui::Dummy({0,kP});ImGui::Indent(kP);
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("연결된 참조점");ImGui::PopStyleColor();
        if(t.point_ref_constraints.empty()){ImGui::PushStyleColor(ImGuiCol_Text,kG50);ImGui::TextUnformatted("(없음)");ImGui::PopStyleColor();}
        else{for(size_t i=0;i<t.point_ref_constraints.size();++i){auto&e=t.point_ref_constraints[i];ImGui::PushID((int)i);ImGui::Text(e.selected_is_follower?"이 점->물체%d/정점%d":"물체%d/정점%d->이 점",e.other_obj,e.other_vert);ImGui::SameLine();if(ImGui::SmallButton("제거")&&t.on_point_ref_remove)t.on_point_ref_remove((int)i);ImGui::PopID();}}
        ImGui::Unindent(kP);ImGui::PopStyleVar(2);ImGui::End();return;
    }

    // ═══ Empty state ═════════════════════════════════════════════════
    if(t.mesh_id<0||t.base_color==nullptr){
        ImGui::Dummy({0,kP});ImGui::Indent(kP);
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("물체 리스트");ImGui::PopStyleColor();
        ImGui::Dummy({0,12});
        if(t.object_list.empty()){
            float aw=CW();
            auto ctr=[&](const char* tx,ImVec4 c){ImVec2 sz=ImGui::CalcTextSize(tx);ImGui::SetCursorPosX(kP+(aw-sz.x)/2);ImGui::PushStyleColor(ImGuiCol_Text,c);ImGui::TextUnformatted(tx);ImGui::PopStyleColor();};
            ImGui::Dummy({0,28});
            ctr("아직 물체가 없어요",kG100);ctr("추가할 물체를 클릭하여 선택하세요.",kG60);
            ImGui::Dummy({0,28});
        } else {
            for(const auto& e:t.object_list){
                ImGui::PushID(e.id);
                // gray5 filled row (not the outline/line button), label
                // left-aligned with 16px left padding.
                ImGui::PushStyleColor(ImGuiCol_Button,kG5);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered,kG10);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,kG10);
                ImGui::PushStyleVar(ImGuiStyleVar_ButtonTextAlign,ImVec2(0.0f,0.5f));
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,ImVec2(16,0));
                ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize,0.0f);
                if(ImGui::Button(e.label.c_str(),{CW(),36})&&t.on_select_object)t.on_select_object(e.id);
                ImGui::PopStyleVar(3);
                ImGui::PopStyleColor(3);
                ImGui::PopID();
                ImGui::Dummy({0,4});
            }
        }
        ImGui::Dummy({0,16});
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("새 물체 추가");ImGui::PopStyleColor();ImGui::Dummy({0,8});
        if(CardButton("obj","3D 모델 파일 불러오기",IcoPlus)&&t.on_request_add_import)t.on_request_add_import();ImGui::Dummy({0,4});
        if(CardButton("cube","정육면체",IcoCube)&&t.on_request_add_cube)t.on_request_add_cube();ImGui::Dummy({0,4});
        if(CardButton("sphere","구",IcoSphere)&&t.on_request_add_sphere)t.on_request_add_sphere();ImGui::Dummy({0,4});
        if(CardButton("cyl","원기둥",IcoCyl)&&t.on_request_add_cylinder)t.on_request_add_cylinder();ImGui::Dummy({0,4});
        if(CardButton("plane","평면",IcoPlane)&&t.on_request_add_plane)t.on_request_add_plane();ImGui::Dummy({0,4});
        if(CardButton("kin","BVH 모션 (키네마틱)",IcoSkeleton)&&t.on_request_add_kinematic)t.on_request_add_kinematic();
        ImGui::Unindent(kP);ImGui::PopStyleVar(2);ImGui::End();return;
    }

    // ═══ Object selected ═════════════════════════════════════════════

    // Header: NO InvisibleButton — draw bg, then place real widgets directly
    {
        float hH=80;
        ImVec2 hP=ImGui::GetCursorScreenPos();
        ImDrawList* dl=ImGui::GetWindowDrawList();
        dl->AddRectFilled(hP,{hP.x+winW,hP.y+hH},IM_COL32(255,255,255,255));

        // Title text
        char hdr[64];std::snprintf(hdr,64,"Mesh #%d",t.mesh_id);
        ImFont* fo=ImGui::GetFont();float tFS=ImGui::GetFontSize();
        dl->AddText(fo,tFS,{hP.x+kP,hP.y+(hH-tFS)/2},ImGui::ColorConvertFloat4ToU32(kG100),hdr);

        float rX=hP.x+winW-kP;

        // Trash button: 40x40
        if(t.on_delete){
            float bW=40,bH=40,bX=rX-bW,bY=hP.y+(hH-bH)/2;
            ImGui::SetCursorScreenPos({bX,bY});
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding,8);ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize,0);
            ImGui::PushStyleColor(ImGuiCol_Button,kG10);ImGui::PushStyleColor(ImGuiCol_ButtonHovered,kG20);ImGui::PushStyleColor(ImGuiCol_ButtonActive,kG20);
            if(ImGui::Button("##tr",{bW,bH}))t.on_delete(t.mesh_id);
            ImGui::PopStyleColor(3);ImGui::PopStyleVar(2);
            DrawTrash(dl,bX+20,bY+20);
            rX=bX-8;
        }

        // Segment tab: 124x40 — use simple buttons, NOT SegmentTab helper
        if(t.current_behavior_index>=0&&t.on_behavior_change){
            int sI=simp(t.current_behavior_index);
            float tabW=124,tabH=40,pad=4,segH=32;
            float segW=(tabW-pad*3)/2;
            float tabX=rX-tabW,tabY=hP.y+(hH-tabH)/2;
            // Tab container bg
            dl->AddRectFilled({tabX,tabY},{tabX+tabW,tabY+tabH},ImGui::ColorConvertFloat4ToU32(kG10),8);

            float textH=ImGui::GetFontSize();float fpy=(segH-textH)*0.5f;
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding,8);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize,0);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,{0,fpy});

            // 강체 button
            bool rigidActive=(sI==1);
            ImGui::SetCursorScreenPos({tabX+pad,tabY+pad});
            ImGui::PushStyleColor(ImGuiCol_Button,rigidActive?kW:ImVec4(0,0,0,0));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,rigidActive?kW:ImVec4(1,1,1,.5f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,kW);
            ImGui::PushStyleColor(ImGuiCol_Text,rigidActive?kG90:kG50);
            if(ImGui::Button("강체",{segW,segH})){
                t.on_behavior_change(t.mesh_id,3); // 3=Rigid
            }
            ImGui::PopStyleColor(4);

            // 옷감 button
            bool clothActive=(sI==0);
            ImGui::SetCursorScreenPos({tabX+pad+segW+pad,tabY+pad});
            ImGui::PushStyleColor(ImGuiCol_Button,clothActive?kW:ImVec4(0,0,0,0));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,clothActive?kW:ImVec4(1,1,1,.5f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,kW);
            ImGui::PushStyleColor(ImGuiCol_Text,clothActive?kG90:kG50);
            if(ImGui::Button("옷감",{segW,segH})){
                t.on_behavior_change(t.mesh_id,clothBI(t.grid_eligible));
            }
            ImGui::PopStyleColor(4);
            ImGui::PopStyleVar(3);
        }

        // Advance cursor past header
        ImGui::SetCursorScreenPos({hP.x, hP.y+hH});
    }

    // ─── 변환 Transform ──────────────────────────────────────────────
    if(t.transform_position||t.rotation_wxyz||t.scale){
        if(AccordionHeader("변환","Transform")){
            ImGui::Dummy({0,kP});ImGui::Indent(kP);
            if(t.transform_position&&t.on_translate){
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("위치");ImGui::PopStyleColor();ImGui::Dummy({0,4});
                float pp[3]={t.transform_position->x,t.transform_position->y,t.transform_position->z};
                if(InputXYZ("pos",pp))t.on_translate(t.mesh_id,tinym::vec3(pp[0],pp[1],pp[2]));
                ImGui::Dummy({0,20});
            }
            if(t.rotation_wxyz&&t.on_rotate){
                const float cw[4]={t.rotation_wxyz[0],t.rotation_wxyz[1],t.rotation_wxyz[2],t.rotation_wxyz[3]};
                float deg[3];quatWxyzToEulerXYZDeg(cw,deg);
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("회전 (각도)");ImGui::PopStyleColor();ImGui::Dummy({0,4});
                if(InputXYZ("rot",deg)){float nw[4];eulerXYZDegToQuatWxyz(deg,nw);t.on_rotate(t.mesh_id,nw[0],nw[1],nw[2],nw[3]);}
                ImGui::Dummy({0,20});
            }
            if(t.scale&&t.on_scale){
                float s[3]={t.scale->x,t.scale->y,t.scale->z};
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("스케일");ImGui::PopStyleColor();ImGui::Dummy({0,4});
                if(InputXYZ("scale",s))t.on_scale(t.mesh_id,tinym::vec3(s[0],s[1],s[2]));
            }
            ImGui::Unindent(kP);ImGui::Dummy({0,kP});
        }
    }

    // ─── 모션 Kinematic playback / motion graphs ─────────────────────
    // Mode-exclusive sub-panels: the combo picks the pose source and only
    // that mode's widgets render (단일 클립 = the original playback UI,
    // untouched in behavior; the two graph modes add build controls and
    // reveal their playback widgets once a build succeeds).
    if(t.kin_panel){
        if(AccordionHeader("모션","BVH Motion")){
            ImGui::Dummy({0,kP});ImGui::Indent(kP);

            // Camera-follow toggle (applies in every playback mode): the
            // viewport orbit pivot rides this body's animated root.
            if(t.on_kin_camera_follow){
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("카메라 추적");ImGui::PopStyleColor();
                ImGui::SameLine(kP+CW()-42);
                bool cf=t.kin_camera_follow;
                if(PillToggle("##kcam",&cf))t.on_kin_camera_follow(t.mesh_id,cf);
                ImGui::Dummy({0,16});
            }

            static const char* kKinModes[7]={
                "단일 클립 재생","랜덤 워크 · 모션 그래프","모션 전환 · 모션 그래프",
                "모션 전환 · DTW","모션 블렌드 스페이스","모션 2-블렌드 · 키타임",
                "모션 N-블렌드 · 키타임"};
            const int mode=t.kin_mode<0?0:(t.kin_mode>6?6:t.kin_mode);
            ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("재생 모드");ImGui::PopStyleColor();ImGui::Dummy({0,4});
            ImGui::SetNextItemWidth(CW());
            if(ImGui::BeginCombo("##kinmode",kKinModes[mode])){
                for(int mi=0;mi<7;++mi){
                    bool sel=(mi==mode);
                    if(ImGui::Selectable(kKinModes[mi],sel)&&!sel&&t.on_kin_mode)
                        t.on_kin_mode(t.mesh_id,mi);
                    if(sel)ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::Dummy({0,16});

            // Shared widget rows; each mode composes the subset it needs.
            auto playRow=[&](bool withLoop){
                bool playing=t.kin_playing;
                if(playing){ImGui::PushStyleColor(ImGuiCol_Button,kG90);ImGui::PushStyleColor(ImGuiCol_Text,kW);}
                if(ImGui::Button(playing?"일시정지":"재생",{96,36})&&t.on_kin_play)
                    t.on_kin_play(t.mesh_id,!playing);
                if(playing)ImGui::PopStyleColor(2);
                if(withLoop){
                    ImGui::SameLine(0,16);
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("반복");ImGui::PopStyleColor();
                    ImGui::SameLine();
                    bool lp=t.kin_loop;
                    if(PillToggle("##kloop",&lp)&&t.on_kin_loop)t.on_kin_loop(t.mesh_id,lp);
                }
            };
            auto scrubRow=[&](){
                float tm=t.kin_time;
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);
                ImGui::Text("시간  %.2fs / %.2fs",tm,t.kin_duration);
                ImGui::PopStyleColor();ImGui::Dummy({0,4});
                ImGui::SetNextItemWidth(CW());
                if(ImGui::SliderFloat("##ktime",&tm,0.0f,t.kin_duration>0?t.kin_duration:1.0f,"")&&t.on_kin_scrub)
                    t.on_kin_scrub(t.mesh_id,tm);
            };
            auto speedRow=[&](){
                // 0.25 step, 0.25–2.0, dropdown.
                static const float kSpeeds[]={0.25f,0.5f,0.75f,1.0f,1.25f,1.5f,1.75f,2.0f};
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("재생 속도");ImGui::PopStyleColor();
                ImGui::SameLine(kP+60);
                char cur[8]; snprintf(cur,sizeof(cur),"%.2fx",t.kin_speed);
                ImGui::SetNextItemWidth(CW()-60);
                if(ImGui::BeginCombo("##kspeed",cur)){
                    for(float s:kSpeeds){
                        char lbl[8]; snprintf(lbl,sizeof(lbl),"%.2fx",s);
                        bool sel=std::abs(t.kin_speed-s)<0.01f;
                        if(ImGui::Selectable(lbl,sel)&&t.on_kin_speed)t.on_kin_speed(t.mesh_id,s);
                        if(sel)ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            };
            auto thresholdRow=[&](){
                float th=t.kin_threshold;
                if(InlineSlider("전환 임계값",&th,0.02f,0.50f)&&t.on_kin_threshold)
                    t.on_kin_threshold(t.mesh_id,th);
            };
            auto markerFracRow=[&](){
                float mf=t.kin_marker_frac;
                if(InlineSlider("관절 방향 가중치",&mf,0.0f,0.30f)&&t.on_kin_marker_frac)
                    t.on_kin_marker_frac(t.mesh_id,mf);
            };
            auto fileCombo=[&](const char* id,const std::string& cur,
                               const std::function<void(const std::string&)>& onPick){
                ImGui::SetNextItemWidth(CW());
                if(ImGui::BeginCombo(id,cur.c_str())){
                    for(const auto& f:t.kin_file_list){
                        bool sel=(f==cur);
                        if(ImGui::Selectable(f.c_str(),sel)&&!sel)onPick(f);
                        if(sel)ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            };
            // ── Reusable motion-clip selector row ────────────────────────
            // One uniform widget for picking a clip in ANY mode: label + file
            // combo + preview-strobe toggle + in-place color swatch + one-shot
            // play button. Drives t.kin_clip_slots[idx] via the slot callbacks.
            // To change the selector everywhere, edit only this lambda; to add
            // it to a new mode, call clipSlotRow(i) per slot.
            auto clipSlotRow=[&](int idx){
                if(idx<0||idx>=(int)t.kin_clip_slots.size())return;
                auto& s=t.kin_clip_slots[idx];
                if(!s.label.empty()){
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted(s.label.c_str());ImGui::PopStyleColor();ImGui::Dummy({0,4});
                }
                char cid[24]; snprintf(cid,sizeof cid,"##slf%d",idx);
                fileCombo(cid,s.file,[&,idx](const std::string& f){
                    if(t.on_kin_slot_file)t.on_kin_slot_file(t.mesh_id,idx,f);
                });
                ImGui::Dummy({0,6});
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("프리뷰");ImGui::PopStyleColor();
                ImGui::SameLine(0,8);
                bool pv=s.preview;
                char tid[24]; snprintf(tid,sizeof tid,"##slpv%d",idx);
                if(PillToggle(tid,&pv)&&t.on_kin_slot_preview)t.on_kin_slot_preview(t.mesh_id,idx,pv);
                if(s.color){
                    ImGui::SameLine(0,10);
                    char colid[24]; snprintf(colid,sizeof colid,"##slc%d",idx);
                    ImGui::ColorEdit3(colid,s.color,ImGuiColorEditFlags_NoInputs|ImGuiColorEditFlags_NoLabel);
                }
                ImGui::SameLine(0,10);
                if(t.kin_preview_playing==idx+1){
                    // This slot is playing → 중단 removes it.
                    char sid[24]; snprintf(sid,sizeof sid,"중단##slpl%d",idx);
                    if(ImGui::SmallButton(sid)&&t.on_kin_preview_stop)t.on_kin_preview_stop(t.mesh_id);
                }else{
                    char pid[24]; snprintf(pid,sizeof pid,"재생##slpl%d",idx);
                    ImGui::BeginDisabled(!(s.preview&&t.kin_sim_paused));
                    if(ImGui::SmallButton(pid)&&t.on_kin_slot_play)t.on_kin_slot_play(t.mesh_id,idx);
                    ImGui::EndDisabled();
                }
                // Active-frame window — colored span = frames the effect+preview
                // use; its first frame anchors the body. Known once the clip is
                // cached (프리뷰 on), so the slider appears with the preview.
                if(s.frame_count>1){
                    ImGui::Dummy({0,8});
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("프레임 범위");ImGui::PopStyleColor();
                    ImGui::SameLine(0,8);
                    ImGui::Text("%d–%d / %d",s.range_start,s.range_end,s.frame_count-1);
                    int lo=s.range_start, hi=s.range_end;
                    char rid[24]; snprintf(rid,sizeof rid,"##slrg%d",idx);
                    if(RangeSliderInt2(rid,&lo,&hi,0,s.frame_count-1)&&t.on_kin_slot_range)
                        t.on_kin_slot_range(t.mesh_id,idx,lo,hi);
                }
                // 루프 연장 (verb modes only): append a copy of the clip so the
                // frame range AND the keytimes span two loops (place a keytime
                // past the original end, into the 2nd loop). Re-detects live.
                if((mode==5||mode==6)&&t.on_kin_slot_loop){
                    ImGui::Dummy({0,6});
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("루프 연장 (프레임 2배)");ImGui::PopStyleColor();
                    ImGui::SameLine(0,8);
                    bool lp=(s.loop_sel>0);
                    char lid[24]; snprintf(lid,sizeof lid,"##sllp%d",idx);
                    if(PillToggle(lid,&lp))t.on_kin_slot_loop(t.mesh_id,idx,lp?1:0);
                }
            };
            auto statusRow=[&](){
                if(t.kin_status.empty())return;
                ImGui::Dummy({0,8});
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX()+CW());
                ImGui::TextUnformatted(t.kin_status.c_str());
                ImGui::PopTextWrapPos();ImGui::PopStyleColor();
            };
            auto labelRow=[&](){
                if(t.kin_label.empty())return;
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("현재");ImGui::PopStyleColor();
                ImGui::SameLine(0,12);
                ImGui::TextUnformatted(t.kin_label.c_str());
                ImGui::Dummy({0,8});
            };

            if(mode==0){
                // 단일 클립 — original playback widgets.
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("모션 파일");ImGui::PopStyleColor();ImGui::Dummy({0,4});
                fileCombo("##kinfile",t.kin_file,[&](const std::string& f){
                    if(t.on_kin_file)t.on_kin_file(t.mesh_id,f);
                });
                ImGui::Dummy({0,16});
                playRow(true);
                ImGui::Dummy({0,16});
                scrubRow();
                ImGui::Dummy({0,12});
                speedRow();
            }else if(mode==1){
                // 랜덤 워크 — clip set → build → walk controls.
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("그래프 클립");ImGui::PopStyleColor();
                ImGui::SameLine(0,12);
                if(ImGui::SmallButton("전체")&&t.on_kin_graph_all)t.on_kin_graph_all(t.mesh_id,true);
                ImGui::SameLine(0,6);
                if(ImGui::SmallButton("해제")&&t.on_kin_graph_all)t.on_kin_graph_all(t.mesh_id,false);
                ImGui::Dummy({0,4});
                if(ImGui::BeginChild("##kinclips",{CW(),168},true)){
                    for(size_t fi=0;fi<t.kin_file_list.size();++fi){
                        bool on=fi<t.kin_graph_selected.size()&&t.kin_graph_selected[fi];
                        if(ImGui::Checkbox(t.kin_file_list[fi].c_str(),&on)&&t.on_kin_graph_toggle)
                            t.on_kin_graph_toggle(t.mesh_id,t.kin_file_list[fi],on);
                    }
                }
                ImGui::EndChild();
                ImGui::Dummy({0,12});
                thresholdRow();
                ImGui::Dummy({0,8});
                markerFracRow();
                ImGui::Dummy({0,12});
                if(ImGui::Button("그래프 빌드",{CW(),36})&&t.on_kin_walk_build)
                    t.on_kin_walk_build(t.mesh_id);
                statusRow();
                if(t.kin_graph_ready){
                    ImGui::Dummy({0,16});
                    labelRow();
                    playRow(false);
                    ImGui::SameLine(0,16);
                    if(ImGui::Button("다른 경로",{96,36})&&t.on_kin_walk_reseed)
                        t.on_kin_walk_reseed(t.mesh_id);
                    ImGui::Dummy({0,12});
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);
                    ImGui::Text("시간  %.1fs",t.kin_time);
                    ImGui::PopStyleColor();
                    ImGui::Dummy({0,12});
                    speedRow();
                }
            }else if(mode==2){
                // 모션 전환 — A → B composite → playback with scrub. Clip pick
                // uses the shared selector (combo + preview + color + play).
                clipSlotRow(0);
                ImGui::Dummy({0,12});
                clipSlotRow(1);
                ImGui::Dummy({0,12});
                thresholdRow();
                ImGui::Dummy({0,8});
                markerFracRow();
                ImGui::Dummy({0,12});
                if(ImGui::Button("전환 생성",{CW(),36})&&t.on_kin_trans_build)
                    t.on_kin_trans_build(t.mesh_id);
                statusRow();
                if(t.kin_graph_ready){
                    ImGui::Dummy({0,16});
                    labelRow();
                    playRow(true);
                    ImGui::Dummy({0,16});
                    scrubRow();
                    ImGui::Dummy({0,12});
                    speedRow();
                }
            }else if(mode==3){
                // 모션 블렌드 — DTW timewarp A→B → time-scaled crossfade track.
                // Clip pick uses the shared selector — each motion's strobe
                // ghost + color + play can be eyeballed before 블렌드 생성.
                clipSlotRow(0);
                ImGui::Dummy({0,12});
                clipSlotRow(1);
                ImGui::Dummy({0,12});
                markerFracRow();
                ImGui::Dummy({0,12});
                if(ImGui::Button("전환 생성",{CW(),36})&&t.on_kin_blend_build)
                    t.on_kin_blend_build(t.mesh_id);
                statusRow();
                if(t.kin_graph_ready){
                    ImGui::Dummy({0,16});
                    labelRow();
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("소스 색 섞기");ImGui::PopStyleColor();
                    ImGui::SameLine(0,10);
                    bool bc=t.kin_blend_colorize;
                    if(PillToggle("##kbcol",&bc)&&t.on_kin_blend_colorize)t.on_kin_blend_colorize(t.mesh_id,bc);
                    ImGui::Dummy({0,12});
                    playRow(true);
                    ImGui::Dummy({0,16});
                    scrubRow();
                    ImGui::Dummy({0,12});
                    speedRow();
                }
            }else if(mode==4){
                // 모션 블렌드 스페이스 — N locomotion clips placed in a 2D pad,
                // blended live by a draggable cursor. The mix drives the
                // kinematic body, which in turn drives any attached cloth.
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX()+CW());
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);
                ImGui::TextUnformatted("프리셋을 고르면 모션 4개가 채워짐. 각 모션은 아래에서 교체·프리뷰, 생성 후 패드로 실시간 혼합.");
                ImGui::PopStyleColor();ImGui::PopTextWrapPos();
                ImGui::Dummy({0,12});
                // Preset dropdown — ABOVE the per-clip combos. "자율선택" = manual.
                // Picking a preset fills the 4 clips; editing any clip reverts here.
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("프리셋");ImGui::PopStyleColor();ImGui::Dummy({0,4});
                const char* curPreset=(t.kin_blend_preset>=0&&t.kin_blend_preset<(int)t.kin_blend_presets.size())
                    ? t.kin_blend_presets[t.kin_blend_preset].c_str() : "자율선택";
                ImGui::SetNextItemWidth(CW());
                if(ImGui::BeginCombo("##bspreset",curPreset)){
                    bool selC=(t.kin_blend_preset<0);
                    if(ImGui::Selectable("자율선택",selC)&&!selC&&t.on_kin_blend_preset)
                        t.on_kin_blend_preset(t.mesh_id,-1);
                    for(int i=0;i<(int)t.kin_blend_presets.size();++i){
                        bool sel=(i==t.kin_blend_preset);
                        if(ImGui::Selectable(t.kin_blend_presets[i].c_str(),sel)&&!sel&&t.on_kin_blend_preset)
                            t.on_kin_blend_preset(t.mesh_id,i);
                        if(sel)ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
                ImGui::Dummy({0,14});
                // Same per-clip selector as the other modes, one row per corner.
                for(int i=0;i<(int)t.kin_clip_slots.size();++i){
                    clipSlotRow(i);
                    ImGui::Dummy({0,10});
                }
                // Build-time root mode: 절대(absolute) blends real per-clip root
                // motion; off = 상대(relative) pin+integrate. Toggling rebuilds.
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("절대 루트");ImGui::PopStyleColor();
                ImGui::SameLine(0,10);
                bool ar=t.kin_blend_absroot;
                if(PillToggle("##kbabs",&ar)&&t.on_kin_blend_absroot)t.on_kin_blend_absroot(t.mesh_id,ar);
                ImGui::Dummy({0,12});
                if(ImGui::Button("블렌드 스페이스 생성",{CW(),36})&&t.on_kin_blendspace_build)
                    t.on_kin_blendspace_build(t.mesh_id);
                statusRow();
                if(t.kin_graph_ready&&!t.kin_blend_coords.empty()){
                    ImGui::Dummy({0,16});
                    // 2D pad: blend coords (in [-1,1]) → a square; drag the
                    // cursor anywhere inside to set the live mix.
                    const float pad=CW();
                    const ImVec2 org=ImGui::GetCursorScreenPos();
                    ImGui::InvisibleButton("##blendpad",{pad,pad});
                    const bool active=ImGui::IsItemActive();
                    ImDrawList* dl=ImGui::GetWindowDrawList();
                    auto N2S=[&](float x,float y){
                        return ImVec2(org.x+(x*0.5f+0.5f)*pad,
                                      org.y+(-y*0.5f+0.5f)*pad);
                    };
                    dl->AddRectFilled(org,{org.x+pad,org.y+pad},IM_COL32(20,20,24,255),8.0f);
                    dl->AddRect(org,{org.x+pad,org.y+pad},IM_COL32(80,80,90,255),8.0f);
                    dl->AddLine(N2S(-1,0),N2S(1,0),IM_COL32(48,48,56,255));
                    dl->AddLine(N2S(0,-1),N2S(0,1),IM_COL32(48,48,56,255));
                    // Clip sample points sized/tinted by their current weight.
                    for(size_t i=0;i<t.kin_blend_coords.size();++i){
                        const ImVec2 pp=N2S(t.kin_blend_coords[i][0],t.kin_blend_coords[i][1]);
                        const float w=i<t.kin_blend_weights.size()?t.kin_blend_weights[i]:0.0f;
                        const ImU32 col=IM_COL32((int)(110+140*w),(int)(120+50*w),(int)(160-70*w),255);
                        dl->AddCircleFilled(pp,4.0f+9.0f*w,col);
                        const char* nm=i<t.kin_blend_labels.size()?t.kin_blend_labels[i].c_str():"";
                        dl->AddText({pp.x+9,pp.y-7},IM_COL32(220,220,228,255),nm);
                    }
                    // Cursor: follow the mouse while the pad is held.
                    ImVec2 cur=N2S(t.kin_blend_cursor[0],t.kin_blend_cursor[1]);
                    if(active){
                        const ImVec2 mp=ImGui::GetIO().MousePos;
                        float nx=((mp.x-org.x)/pad)*2.0f-1.0f;
                        float ny=-(((mp.y-org.y)/pad)*2.0f-1.0f);
                        nx=nx<-1?-1:(nx>1?1:nx); ny=ny<-1?-1:(ny>1?1:ny);
                        if(t.on_kin_blend_cursor)t.on_kin_blend_cursor(t.mesh_id,nx,ny);
                        cur=N2S(nx,ny);
                    }
                    dl->AddCircle(cur,9.0f,IM_COL32(255,255,255,255),0,2.0f);
                    dl->AddCircleFilled(cur,3.0f,IM_COL32(255,255,255,255));
                    ImGui::Dummy({0,12});
                    // 1D bars, one per axis. Each edits ONLY its own axis and
                    // passes the other through unchanged (moving one bar never
                    // disturbs the other or the pad). Labelled by the clip files
                    // at the axis ends — no fixed role names. Grey frame bg to
                    // match the material sliders (InlineSlider).
                    auto nearestLabel=[&](float tx,float ty)->const char*{
                        int best=-1; float bd=1e9f;
                        for(size_t i=0;i<t.kin_blend_coords.size();++i){
                            float dx=t.kin_blend_coords[i][0]-tx,dy=t.kin_blend_coords[i][1]-ty;
                            float d=dx*dx+dy*dy;
                            if(d<bd){bd=d;best=(int)i;}
                        }
                        return (best>=0&&best<(int)t.kin_blend_labels.size())?t.kin_blend_labels[best].c_str():"";
                    };
                    auto axisBar=[&](const char* id,const char* lbl,float val,
                                     const std::function<void(float)>& onChange){
                        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted(lbl);ImGui::PopStyleColor();ImGui::Dummy({0,4});
                        ImGui::PushStyleColor(ImGuiCol_FrameBg,kG10);ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,kG10);ImGui::PushStyleColor(ImGuiCol_FrameBgActive,kG10);
                        ImGui::SetNextItemWidth(CW());
                        if(ImGui::SliderFloat(id,&val,-1.0f,1.0f,""))onChange(val);
                        ImGui::PopStyleColor(3);
                    };
                    char ylbl[128],xlbl[128];
                    snprintf(ylbl,sizeof ylbl,"%s  ↔  %s",nearestLabel(0,-1),nearestLabel(0,1));
                    snprintf(xlbl,sizeof xlbl,"%s  ↔  %s",nearestLabel(-1,0),nearestLabel(1,0));
                    axisBar("##bswr",ylbl,t.kin_blend_cursor[1],[&](float v){
                        if(t.on_kin_blend_cursor)t.on_kin_blend_cursor(t.mesh_id,t.kin_blend_cursor[0],v);
                    });
                    ImGui::Dummy({0,8});
                    axisBar("##bssl",xlbl,t.kin_blend_cursor[0],[&](float v){
                        if(t.on_kin_blend_cursor)t.on_kin_blend_cursor(t.mesh_id,v,t.kin_blend_cursor[1]);
                    });
                    ImGui::Dummy({0,12});
                    // Blend-result preview: strobe ghost of the blended cycle
                    // (toggle), plus a one-shot 재생 of the blend through one
                    // cycle. Both morph live as the pad/slider moves; 재생 is
                    // paused-only.
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("블렌드 프리뷰");ImGui::PopStyleColor();
                    ImGui::SameLine(0,10);
                    bool bp=t.kin_blend_preview;
                    if(PillToggle("##kbpv",&bp)&&t.on_kin_blend_preview)t.on_kin_blend_preview(t.mesh_id,bp);
                    ImGui::SameLine(0,12);
                    if(t.kin_preview_playing==-1){
                        if(ImGui::SmallButton("중단##kbplay")&&t.on_kin_preview_stop)t.on_kin_preview_stop(t.mesh_id);
                    }else{
                        ImGui::BeginDisabled(!t.kin_sim_paused);
                        if(ImGui::SmallButton("재생##kbplay")&&t.on_kin_blend_play)t.on_kin_blend_play(t.mesh_id);
                        ImGui::EndDisabled();
                    }
                    ImGui::Dummy({0,12});
                    labelRow();
                    playRow(false);
                    ImGui::Dummy({0,12});
                    speedRow();
                }
            }else if(mode==5||mode==6){
                // 모션 키타임 블렌드 (Verbs & Adverbs) — N개 모션을 발
                // 키타임(L/R 착지·이륙)으로 같은 phase에 맞춘 뒤, 태그(부사)
                // 값으로 RBF 블렌드. 발 키타임은 자동 검출 후 수정 가능.
                // mode 5 = 정확히 두 모션, mode 6 = 모션 추가/삭제(2..N).
                const bool nMode=(mode==6);
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX()+CW());
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);
                ImGui::TextUnformatted(nMode
                    ?"N개 모션을 발 키타임으로 정렬해 같은 동작 순간끼리 블렌드. 모션을 추가/삭제하고 각 모션을 프리뷰로 켜 프레임 범위를 정한 뒤 생성하면 키타임이 자동 검출됨."
                    :"두 모션을 발 키타임으로 정렬해 같은 동작 순간끼리 블렌드. 각 모션을 프리뷰로 켜 프레임 범위를 정한 뒤 생성하면 키타임이 자동 검출됨(이후 수정 가능).");
                ImGui::PopStyleColor();ImGui::PopTextWrapPos();
                ImGui::Dummy({0,12});
                // Preset combo (mode 6 only): picking one fills the motions +
                // colors + loop + keytimes + tag + adverbs and builds. 자율선택
                // = manual; a file/motion edit reverts here.
                if(nMode&&!t.kin_verb_presets.empty()){
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("프리셋");ImGui::PopStyleColor();ImGui::Dummy({0,4});
                    const char* curP=(t.kin_verb_preset>=0&&t.kin_verb_preset<(int)t.kin_verb_presets.size())
                        ? t.kin_verb_presets[t.kin_verb_preset].c_str() : "자율선택";
                    ImGui::SetNextItemWidth(CW());
                    if(ImGui::BeginCombo("##vbpreset",curP)){
                        bool selC=(t.kin_verb_preset<0);
                        if(ImGui::Selectable("자율선택",selC)&&!selC&&t.on_kin_verb_preset)
                            t.on_kin_verb_preset(t.mesh_id,-1);
                        for(int i=0;i<(int)t.kin_verb_presets.size();++i){
                            bool sel=(i==t.kin_verb_preset);
                            if(ImGui::Selectable(t.kin_verb_presets[i].c_str(),sel)&&!sel&&t.on_kin_verb_preset)
                                t.on_kin_verb_preset(t.mesh_id,i);
                            if(sel)ImGui::SetItemDefaultFocus();
                        }
                        ImGui::EndCombo();
                    }
                    ImGui::Dummy({0,14});
                }
                // File rows. mode 5 = fixed 2; mode 6 = one row per motion with a
                // 삭제 button (when >2) plus a + 모션 추가 button below.
                const int nFiles=nMode?(int)t.kin_clip_slots.size():2;
                for(int i=0;i<nFiles;++i){
                    clipSlotRow(i);
                    // Own line (not SameLine) — clipSlotRow's last widget is a
                    // full-width range slider once the preview caches, which a
                    // trailing SameLine would overrun.
                    if(nMode&&nFiles>2){
                        ImGui::Dummy({0,4});
                        char rid[32]; snprintf(rid,sizeof rid,"모션 %d 삭제##vrm%d",i+1,i);
                        if(ImGui::SmallButton(rid)&&t.on_kin_verb_remove_motion)
                            t.on_kin_verb_remove_motion(t.mesh_id,i);
                    }
                    ImGui::Dummy({0,10});
                }
                if(nMode&&nFiles<4){  // ponytail: 4 = motionSlots preview pool
                    if(ImGui::Button("+ 모션 추가",{CW(),28})&&t.on_kin_verb_add_motion)
                        t.on_kin_verb_add_motion(t.mesh_id,"");
                    ImGui::Dummy({0,8});
                }
                if(ImGui::Button("키타임 검출 · 블렌드 생성",{CW(),36})&&t.on_kin_verb_build)
                    t.on_kin_verb_build(t.mesh_id);
                statusRow();
                if(t.kin_verb_ready){
                    // Motion display name (file stem) by index; falls back to
                    // "모션 N" if the name snapshot is short. Used everywhere a
                    // motion is referenced so the user sees WHICH clip.
                    auto vname=[&](int e)->std::string{
                        if(e>=0&&e<(int)t.kin_verb_names.size()&&!t.kin_verb_names[e].empty())
                            return t.kin_verb_names[e];
                        return "모션 "+std::to_string(e+1);
                    };
                    // ── Editable keytimes, one block per motion ──
                    static const char* kKT[5]={
                        "왼발 착지","오른발 이륙","오른발 착지","왼발 이륙","주기 끝"};
                    ImGui::Dummy({0,16});
                    for(int e=0;e<(int)t.kin_verb_keys.size();++e){
                        std::string hdr=vname(e)+" · 키타임(프레임)";
                        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted(hdr.c_str());ImGui::PopStyleColor();
                        // Per-section keytime preview toggle (common to every
                        // motion): ghosts this clip at its 5 keytime frames so you
                        // slide a handle and watch the pose. One section at a time.
                        ImGui::SameLine(0,10);
                        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("프리뷰");ImGui::PopStyleColor();
                        ImGui::SameLine(0,6);
                        bool kp=(t.kin_verb_kt_preview==e);
                        char kpid[20]; snprintf(kpid,sizeof kpid,"##vktpv%d",e);
                        if(PillToggle(kpid,&kp)&&t.on_kin_verb_kt_preview)
                            t.on_kin_verb_kt_preview(t.mesh_id,e,kp);
                        ImGui::Dummy({0,4});
                        const int fc=e<(int)t.kin_verb_frame_count.size()?t.kin_verb_frame_count[e]:1;
                        for(int wkt=0;wkt<5;++wkt){
                            ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted(kKT[wkt]);ImGui::PopStyleColor();
                            ImGui::SameLine(kP+108);
                            int v=t.kin_verb_keys[e][wkt];
                            char id[24]; snprintf(id,sizeof id,"##vkt%d_%d",e,wkt);
                            ImGui::SetNextItemWidth(CW()-108);
                            if(ImGui::DragInt(id,&v,0.5f,0,fc>1?fc-1:1,"%d")&&t.on_kin_verb_keytime)
                                t.on_kin_verb_keytime(t.mesh_id,e,wkt,v);
                            if(wkt<4)ImGui::Dummy({0,4});
                        }
                        // 주기끝 = 왼발착지 + 한 모션(루프) 길이. 루프 연장과 함께
                        // 쓰면 2번째 루프에 착지와 같은 포즈로 떨어져 이음매가 깔끔.
                        ImGui::Dummy({0,6});
                        char ceid[48]; snprintf(ceid,sizeof ceid,"주기끝 = 착지 + 모션 길이##vce%d",e);
                        if(ImGui::SmallButton(ceid)&&t.on_kin_verb_cycle_end)
                            t.on_kin_verb_cycle_end(t.mesh_id,e);
                        ImGui::Dummy({0,12});
                    }
                    // ── Tags (adverbs): 1~2개. 추가하면 모션별 % 필드가 생김 ──
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("태그 (부사)");ImGui::PopStyleColor();
                    if((int)t.kin_verb_tags.size()<2){
                        ImGui::SameLine(0,10);
                        if(ImGui::SmallButton("+ 태그 추가")&&t.on_kin_verb_add_tag)
                            t.on_kin_verb_add_tag(t.mesh_id,"");
                    }
                    ImGui::Dummy({0,8});
                    for(int tg=0;tg<(int)t.kin_verb_tags.size();++tg){
                        char nm[64]={0};
                        snprintf(nm,sizeof nm,"%s",t.kin_verb_tags[tg].c_str());
                        ImGui::SetNextItemWidth((int)t.kin_verb_tags.size()>1?CW()-64:CW());
                        char nid[16]; snprintf(nid,sizeof nid,"##vtn%d",tg);
                        if(ImGui::InputText(nid,nm,sizeof nm)&&t.on_kin_verb_tag_name)
                            t.on_kin_verb_tag_name(t.mesh_id,tg,nm);
                        if((int)t.kin_verb_tags.size()>1){
                            ImGui::SameLine(0,6);
                            char rid[20]; snprintf(rid,sizeof rid,"삭제##vtr%d",tg);
                            if(ImGui::SmallButton(rid)&&t.on_kin_verb_remove_tag)
                                t.on_kin_verb_remove_tag(t.mesh_id,tg);
                        }
                        ImGui::Dummy({0,4});
                        const int nA=(int)t.kin_verb_adverb.size();
                        for(int e=0;e<nA;++e){
                            // Motion name on its own line (variable-width file
                            // stems overflow a fixed inline column), slider below.
                            ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted(vname(e).c_str());ImGui::PopStyleColor();
                            float val=t.kin_verb_adverb[e][tg];
                            ImGui::SetNextItemWidth(CW());
                            char vid[24]; snprintf(vid,sizeof vid,"##vadv%d_%d",e,tg);
                            if(ImGui::SliderFloat(vid,&val,0.0f,100.0f,"%.0f%%")&&t.on_kin_verb_adverb)
                                t.on_kin_verb_adverb(t.mesh_id,e,tg,val);
                            if(e+1<nA)ImGui::Dummy({0,6});
                        }
                        ImGui::Dummy({0,12});
                    }
                    // 외삽 허용: 끄면 두 모션 사이로 제한(convex), 켜면 모션을
                    // 지나 과장(extrapolation, 부호 있는 RBF 가중치). 1태그 슬라이더
                    // 범위가 -50~150%로 넓어져 외삽 지점에 도달 가능.
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("외삽 허용");ImGui::PopStyleColor();
                    ImGui::SameLine(0,10);
                    bool ve=t.kin_verb_extrapolate;
                    if(PillToggle("##kvext",&ve)&&t.on_kin_verb_extrapolate)t.on_kin_verb_extrapolate(t.mesh_id,ve);
                    ImGui::Dummy({0,12});
                    // ── Blend query: 1태그=슬라이더, 2태그=평면 패드 ──
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("블렌드 위치");ImGui::PopStyleColor();ImGui::Dummy({0,6});
                    if((int)t.kin_verb_tags.size()<=1){
                        const char* tn=t.kin_verb_tags.empty()?"":t.kin_verb_tags[0].c_str();
                        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted(tn);ImGui::PopStyleColor();ImGui::Dummy({0,2});
                        float q=t.kin_verb_query[0];
                        const float qlo=t.kin_verb_extrapolate?-50.0f:0.0f;
                        const float qhi=t.kin_verb_extrapolate?150.0f:100.0f;
                        ImGui::SetNextItemWidth(CW());
                        if(ImGui::SliderFloat("##vq0",&q,qlo,qhi,"%.0f%%")&&t.on_kin_verb_query)
                            t.on_kin_verb_query(t.mesh_id,0,q);
                    }else{
                        // 2-tag plane: generator axes are the TAG values, 0~100%.
                        // Horizontal (X) = tag[0], vertical (Y) = tag[1]; dragging
                        // the cursor sets both at once. Spell it out by tag name so
                        // the user knows what each axis adjusts.
                        const std::string t0=t.kin_verb_tags[0], t1=t.kin_verb_tags[1];
                        ImGui::PushTextWrapPos(ImGui::GetCursorPosX()+CW());
                        ImGui::PushStyleColor(ImGuiCol_Text,kG60);
                        std::string cap="가로(X) = "+t0+"  ·  세로(Y) = "+t1+"   (각 0~100%, 오른쪽·위로 갈수록 값↑)";
                        ImGui::TextUnformatted(cap.c_str());
                        ImGui::PopStyleColor();ImGui::PopTextWrapPos();
                        ImGui::Dummy({0,6});
                        const float pad=CW();
                        const ImVec2 org=ImGui::GetCursorScreenPos();
                        ImGui::InvisibleButton("##vpad",{pad,pad});
                        const bool actv=ImGui::IsItemActive();
                        ImDrawList* dl=ImGui::GetWindowDrawList();
                        auto P2S=[&](float x,float y){return ImVec2(org.x+(x/100.f)*pad,org.y+(1.f-y/100.f)*pad);};
                        dl->AddRectFilled(org,{org.x+pad,org.y+pad},IM_COL32(20,20,24,255),8.f);
                        dl->AddRect(org,{org.x+pad,org.y+pad},IM_COL32(80,80,90,255),8.f);
                        // Axis name labels: tag[1] up the left edge (Y), tag[0]
                        // along the bottom-right (X).
                        const ImU32 axc=IM_COL32(150,150,160,255);
                        dl->AddText({org.x+6,org.y+5},axc,("↑ "+t1).c_str());
                        const ImVec2 xs=ImGui::CalcTextSize(("→ "+t0).c_str());
                        dl->AddText({org.x+pad-xs.x-6,org.y+pad-xs.y-5},axc,("→ "+t0).c_str());
                        for(int e=0;e<(int)t.kin_verb_adverb.size();++e){
                            ImVec2 pp=P2S(t.kin_verb_adverb[e][0],t.kin_verb_adverb[e][1]);
                            float w=e<(int)t.kin_verb_weights.size()?t.kin_verb_weights[e]:0.f;
                            dl->AddCircleFilled(pp,4.f+9.f*w,IM_COL32(120,140,200,255));
                            dl->AddText({pp.x+8,pp.y-7},IM_COL32(220,220,228,255),vname(e).c_str());
                        }
                        ImVec2 cu=P2S(t.kin_verb_query[0],t.kin_verb_query[1]);
                        if(actv){
                            ImVec2 mp=ImGui::GetIO().MousePos;
                            float nx=((mp.x-org.x)/pad)*100.f, ny=(1.f-(mp.y-org.y)/pad)*100.f;
                            nx=nx<0?0:(nx>100?100:nx); ny=ny<0?0:(ny>100?100:ny);
                            if(t.on_kin_verb_query){t.on_kin_verb_query(t.mesh_id,0,nx);t.on_kin_verb_query(t.mesh_id,1,ny);}
                            cu=P2S(nx,ny);
                        }
                        dl->AddCircle(cu,9.f,IM_COL32(255,255,255,255),0,2.f);
                        dl->AddCircleFilled(cu,3.f,IM_COL32(255,255,255,255));
                        // Live query values, named by tag.
                        ImGui::Dummy({0,6});
                        char qv[160]; snprintf(qv,sizeof qv,"현재  %s %.0f%%   ·   %s %.0f%%",
                                               t0.c_str(),t.kin_verb_query[0],t1.c_str(),t.kin_verb_query[1]);
                        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted(qv);ImGui::PopStyleColor();
                    }
                    if(t.kin_verb_weights.size()>=2){
                        ImGui::Dummy({0,8});
                        std::string mix="혼합  ";
                        for(size_t i=0;i<t.kin_verb_weights.size();++i){
                            if(i)mix+="   ·   ";
                            char b[16]; snprintf(b,sizeof b," %.0f%%",t.kin_verb_weights[i]*100.f);
                            mix+=vname((int)i)+b;
                        }
                        ImGui::PushStyleColor(ImGuiCol_Text,kG60);
                        ImGui::TextWrapped("%s",mix.c_str());
                        ImGui::PopStyleColor();
                    }
                    // 블렌드 결과 프리뷰: 라이브 블렌드 사이클 스트로브(토글) +
                    // 한 사이클 1회 재생(일시정지 상태에서만). 슬라이더를 움직이면
                    // 고스트가 실시간으로 다시 칠해지고 다시 포즈됨.
                    ImGui::Dummy({0,14});
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("블렌드 프리뷰");ImGui::PopStyleColor();
                    ImGui::SameLine(0,10);
                    bool vp=t.kin_verb_preview;
                    if(PillToggle("##kvpv",&vp)&&t.on_kin_verb_preview)t.on_kin_verb_preview(t.mesh_id,vp);
                    ImGui::SameLine(0,12);
                    if(t.kin_preview_playing==-1){
                        if(ImGui::SmallButton("중단##kvplay")&&t.on_kin_preview_stop)t.on_kin_preview_stop(t.mesh_id);
                    }else{
                        ImGui::BeginDisabled(!t.kin_sim_paused);
                        if(ImGui::SmallButton("재생##kvplay")&&t.on_kin_verb_play)t.on_kin_verb_play(t.mesh_id);
                        ImGui::EndDisabled();
                    }
                    ImGui::Dummy({0,16});
                    playRow(true);
                    ImGui::Dummy({0,16});
                    // 개방형 재생(이동) — 스크럽 대신 경과 시간 + 처음으로.
                    ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::Text("시간  %.1fs",t.kin_time);ImGui::PopStyleColor();
                    ImGui::SameLine(0,12);
                    if(ImGui::SmallButton("처음으로")&&t.on_kin_scrub)t.on_kin_scrub(t.mesh_id,0.0f);
                    ImGui::Dummy({0,12});
                    speedRow();
                }
            }
            ImGui::Unindent(kP);ImGui::Dummy({0,kP});
        }
    }

    // ─── 머터리얼 Material ───────────────────────────────────────────
    bool hM=t.metallic&&t.roughness&&t.specular_weight&&t.emission_color&&t.on_material_edit;
    auto fM=[&](){if(!hM)return;t.on_material_edit(t.mesh_id,*t.base_color,*t.metallic,*t.roughness,*t.specular_weight,*t.emission_color);};
    if(AccordionHeader("머터리얼","Material")){
        ImGui::Dummy({0,kP});ImGui::Indent(kP);
        ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("RGB");ImGui::PopStyleColor();ImGui::Dummy({0,4});
        if(InputRGB("bc",t.base_color->v))fM();
        if(hM){
            ImGui::Dummy({0,24});
            ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("PBR 속성");ImGui::PopStyleColor();ImGui::Dummy({0,8});
            if(InlineSlider("금속성",t.metallic,0,1))fM();ImGui::Dummy({0,8});
            if(InlineSlider("거칠기",t.roughness,0,1))fM();ImGui::Dummy({0,8});
            if(InlineSlider("스페큘러",t.specular_weight,0,1))fM();
            ImGui::Dummy({0,24});
            ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("발광 색상");ImGui::PopStyleColor();ImGui::Dummy({0,4});
            if(InputRGB("em",t.emission_color->v))fM();
        }
        ImGui::Unindent(kP);ImGui::Dummy({0,kP});
    }

    // ─── 렌더링 Render (plane checkerboard) ──────────────────────────
    if(t.checkerboard&&t.on_checkerboard){
        if(AccordionHeader("렌더링","Render")){
            ImGui::Dummy({0,kP});ImGui::Indent(kP);
            ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("체커보드");ImGui::PopStyleColor();
            ImGui::SameLine(kP+CW()-42);
            if(PillToggle("##chk",t.checkerboard))t.on_checkerboard(t.mesh_id,*t.checkerboard);
            ImGui::Unindent(kP);ImGui::Dummy({0,kP});
        }
    }

    // ─── 팽팽함 ──────────────────────────────────────────────────────
    if(t.cloth_stiffness_scale&&t.on_cloth_stiffness_scale){
        float sc=*t.cloth_stiffness_scale;float k=(sc>0)?std::log10(sc):0;if(k<-2)k=-2;if(k>2)k=2;
        ImGui::Dummy({0,kP});ImGui::Indent(kP);
        if(InlineSlider("팽팽함",&k,-2,2)){float ns=std::pow(10.f,k);*t.cloth_stiffness_scale=ns;t.on_cloth_stiffness_scale(t.mesh_id,ns);}
        // Per-type stiffness coefficients (log10 slider, 1e2..1e7).
        // A null pointer keeps that row hidden — FastGridCloth omits
        // shear, TriangularCloth shows all three.
        auto clothK=[&](const char* lbl,float* val,
                        const std::function<void(int,float)>& cb){
            if(!val||!cb) return;
            float lk=(*val>0)?std::log10(*val):2.f;
            if(lk<2)lk=2;if(lk>7)lk=7;
            if(InlineSlider(lbl,&lk,2,7)){
                float nv=std::pow(10.f,lk);*val=nv;cb(t.mesh_id,nv);}
            ImGui::Dummy({0,8});
        };
        if(t.cloth_stretch){ImGui::Dummy({0,12});
            ImGui::PushStyleColor(ImGuiCol_Text,kG60);
            ImGui::TextUnformatted("강성");ImGui::PopStyleColor();
            ImGui::Dummy({0,8});}
        clothK("스트레치",t.cloth_stretch,t.on_cloth_stretch);
        clothK("시어",    t.cloth_shear,  t.on_cloth_shear);
        clothK("벤드",    t.cloth_bend,   t.on_cloth_bend);
        ImGui::Unindent(kP);ImGui::Dummy({0,kP});
    }

    // ─── 환경 Environment ────────────────────────────────────────────
    if(t.apply_gravity||t.apply_wind||t.is_static){
        if(AccordionHeader("환경","Environment")){
            ImGui::Dummy({0,kP});ImGui::Indent(kP);
            auto fE=[&](){if(!t.on_env_toggle_change)return;bool g=t.apply_gravity?*t.apply_gravity:true;bool w=t.apply_wind?*t.apply_wind:true;t.on_env_toggle_change(t.mesh_id,g,w);};
            if(t.is_static){
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("고정");ImGui::PopStyleColor();
                ImGui::SameLine(kP+CW()-42);if(PillToggle("##st",t.is_static)){if(t.on_static_change)t.on_static_change(t.mesh_id,*t.is_static);}ImGui::Dummy({0,12});
            }
            if(t.apply_gravity){
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("중력");ImGui::PopStyleColor();
                ImGui::SameLine(kP+CW()-42);if(PillToggle("##gv",t.apply_gravity))fE();ImGui::Dummy({0,12});
            }
            if(t.apply_wind){
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("바람");ImGui::PopStyleColor();
                ImGui::SameLine(kP+CW()-42);if(PillToggle("##wd",t.apply_wind))fE();ImGui::Dummy({0,12});
            }
            ImGui::Unindent(kP);ImGui::Dummy({0,kP});
        }
    }

    // ─── 서브오브젝트 BVH Sub-object BVH (per-object; master in profiler) ──
    if(t.subobj_split_s&&t.subobj_render){
        if(AccordionHeader("서브오브젝트 BVH","Sub-object BVH")){
            ImGui::Dummy({0,kP});ImGui::Indent(kP);
            // split s slider (integer 1..8, k = 4^s) — per object
            {
                float contentW=CW(),labelOff=60,valW=36,gap=8,sliderW=contentW-labelOff-gap-valW;
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("분할 s");ImGui::PopStyleColor();
                ImGui::SameLine(kP+labelOff);
                ImGui::PushStyleColor(ImGuiCol_FrameBg,kG10);ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,kG10);ImGui::PushStyleColor(ImGuiCol_FrameBgActive,kG10);
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding,ImVec2(0,4));
                ImGui::SetNextItemWidth(sliderW);
                int si=*t.subobj_split_s;
                bool ch=ImGui::SliderInt("##sob_s",&si,1,8,"");
                ImGui::PopStyleVar();ImGui::PopStyleColor(3);
                ImGui::SameLine(kP+contentW-valW);
                ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::Text("%d",si);ImGui::PopStyleColor();
                if(ch&&si!=*t.subobj_split_s){*t.subobj_split_s=si;if(t.on_subobj_split)t.on_subobj_split(t.mesh_id,si);}
            }
            ImGui::Dummy({0,12});
            // color the selected mesh's triangles by cluster
            ImGui::PushStyleColor(ImGuiCol_Text,kG60);ImGui::TextUnformatted("클러스터 렌더링");ImGui::PopStyleColor();
            ImGui::SameLine(kP+CW()-42);
            if(PillToggle("##sob_rn",t.subobj_render)){if(t.on_subobj_render)t.on_subobj_render(t.mesh_id,*t.subobj_render);}
            ImGui::Unindent(kP);ImGui::Dummy({0,kP});
        }
    }

    ImGui::PopStyleVar(2);ImGui::End();
}
} // namespace mesh_inspector
