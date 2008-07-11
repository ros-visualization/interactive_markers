#include "TopicDisplay.h"

TopicDisplay::TopicDisplay( wxWindow* parent, ros::node* _node ) : GenTopicDisplay( parent ), rosNode(_node)
{
    timer = new wxTimer(this);

    this->Connect( wxEVT_TIMER, wxTimerEventHandler( TopicDisplay::tick ), NULL, this );

    timer->Start(1000);

    rootId  = topicTree->AddRoot(wxT("/"));
}


void TopicDisplay::checkIsTopic ( wxTreeEvent& event )
{
  if (topicTree->GetItemData(event.GetItem()) == NULL)
    event.Veto();
}

void TopicDisplay::tick( wxTimerEvent& event )
{
  TopicList topics;

  rosNode->get_published_topics(&topics);

  // Set all items in cache to tentatively delete
  for (TopicMap::iterator i = topicCache.begin(); i != topicCache.end(); i++)
    i->second.save = false;

  // Loop through all published topics
  for (TopicList::iterator i = topics.begin(); i != topics.end(); i++)
  {
    TopicMap::iterator j = topicCache.find(i->first);
    if (j == topicCache.end())
    {      
      // Topic not in cache yet.  Find right place to put it in the tree
      std::istringstream iss(i->first);
      std::string token;

      wxTreeItemId id = topicTree->GetRootItem();
      
      while (getline(iss, token, '/'))
      {
        if (!token.empty())
        {
          wxTreeItemIdValue cookie;
          wxTreeItemId child = topicTree->GetFirstChild(id,cookie);
          bool exists = false;
          do
            if (topicTree->GetItemText(child) == wxString::FromAscii( token.c_str() ))
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
      }

      // Add to Cache
      TopicMapEntry cacheItem;
      cacheItem.item = id;
      cacheItem.save = true;
      cacheItem.type = i->second;
      topicCache[i->first] = cacheItem;

      // Put data in tree item and rename
      TopicNameData* data = new TopicNameData();
      data->name = i->first;
      topicTree->SetItemText(id, wxString::FromAscii( token.c_str() ) + wxT(" (") + wxString::FromAscii( i->second.c_str() ) + wxT(")"));
      topicTree->SetItemData(id, data);
      topicTree->SetItemBold(id, true);
    } else {
      // Topic already in cache -- keep it there.
      j->second.save = true;
    }
  }

  std::vector<TopicMap::iterator> toErase;

  // Tentatively delete all items in cache which should be removed
  for (TopicMap::iterator i = topicCache.begin(); i != topicCache.end(); i++)
  {
    if (i->second.save == false)
      toErase.push_back(i);
  }

  wxTreeItemId id;
  wxTreeItemId parentId;

  // Actually delete all items and purge parents as necessary
  for (std::vector<TopicMap::iterator>::iterator i = toErase.begin(); i != toErase.end(); i++)
  {
    // Delete item
    id = (*i)->second.item;
    parentId = topicTree->GetItemParent(id);
    topicTree->Delete(id);

    // Delete all childless parents
    while (parentId != rootId)
    {
      if (topicTree->HasChildren(parentId))
        break;
      else
      {
        id = parentId;
        parentId = topicTree->GetItemParent(id);
        topicTree->Delete(id);
      }
    }

    // Erase item from cache
    topicCache.erase(*i);
  }

  // Refresh the display
  Refresh();
}
