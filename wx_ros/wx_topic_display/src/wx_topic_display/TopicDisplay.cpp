#include "TopicDisplay.h"

TopicDisplay::TopicDisplay( wxWindow* parent, ros::node* _node ) : GenTopicDisplay( parent ), rosNode(_node)
{
    timer = new wxTimer(this);

    this->Connect( wxEVT_TIMER, wxTimerEventHandler( TopicDisplay::tick ), NULL, this );

    timer->Start(1000);

    wxTreeItemId root_id = topicTree->AddRoot(wxT("/"));
}


void TopicDisplay::checkIsTopic ( wxTreeEvent& event )
{
  if (topicTree->GetItemData(event.GetItem()) == NULL)
    event.Veto();
}

void TopicDisplay::tick( wxTimerEvent& event )
{
  topicList topics;

  rosNode->get_published_topics(&topics);

  wxTreeItemId root_id = topicTree->GetRootItem();

  bool clean = false;
  int pass = 0;

  while (!clean || pass < 2)
  {
    clean = true;
    wxTreeItemId node = topicTree->GetRootItem();
    wxTreeItemIdValue cookie;

    while (node.IsOk()) {
      printf("Checking: %s\n", (const char*)(topicTree->GetItemText(node)).mb_str(wxConvUTF8));
      if (topicTree->ItemHasChildren(node))
        node = topicTree->GetFirstChild(node, cookie);
      else
      {
        bool needsDeleting = false;

        treeData* data = (treeData*)topicTree->GetItemData(node);

        if (data == NULL)
          needsDeleting = true;
        else
        {
          switch (pass) {
          case 0:
            data->save = false;
            for (topicList::iterator i = topics.begin(); i != topics.end(); i++)
            {
              if ( data->name == i->first )
              {
                data->save = true;
                topics.erase(i);
                break;
              }
            }
            break;
          default:
            if (data->save == false)
              needsDeleting = true;
          }
        }
          
        wxTreeItemId tmp = node;


        wxTreeItemId n_node = topicTree->GetNextSibling(node);
        if (n_node.IsOk())
          node = n_node;
        else
        {
          node = topicTree->GetItemParent(node);

          while (node.IsOk())
          {
            wxTreeItemId n_node = topicTree->GetNextSibling(node);
            if (n_node.IsOk())
            {
              node = n_node;
              break;
            }
            else
              node = topicTree->GetItemParent(node);
          }
        }
   
        if (needsDeleting && tmp != root_id)
        {
          clean = false;
          topicTree->Delete(tmp);
        }
      }
    }

    pass++;
  }

  for (topicList::iterator i = topics.begin(); i != topics.end(); i++)
  {
    std::istringstream iss(i->first);
    std::string token;

    wxTreeItemId id = topicTree->GetRootItem();

    while (getline(iss, token, '/'))
      if (token != std::string(""))
      {
        wxTreeItemIdValue cookie;
        wxTreeItemId child = topicTree->GetFirstChild(id,cookie);
        bool exists = false;
        do
          if (topicTree->GetItemText(child) == wxString::FromAscii( token.c_str()))
          {
            exists = true;
            break;
          }
        while ((child = topicTree->GetNextChild(id,cookie)).IsOk());

        if (exists)
          id = child;
        else
          id = topicTree->AppendItem(id, wxString::FromAscii( token.c_str()  ));        
      }

    treeData* data = new treeData();
    data->name = i->first;
    topicTree->SetItemText(id, wxString::FromAscii( token.c_str() ) + wxT(" (") + wxString::FromAscii( i->second.c_str() ) + wxT(")"));
    topicTree->SetItemData(id, data);
    topicTree->SetItemBold(id, true);
  }
    
  Refresh();
}
