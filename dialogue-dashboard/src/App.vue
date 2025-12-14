<template>
  <v-app>
    <v-navigation-drawer app v-model="drawer">
      <v-container>
        <v-list-item>
          <v-list-item-content>
            <v-list-item-title class="title">Menu</v-list-item-title>
          </v-list-item-content>
        </v-list-item>
        
        <v-divider/>

        <v-list nav>
          <v-list-item v-for="list in lists" :key="list.name" :to="list.link">
            <v-list-item-icon>
              <v-icon>{{ list.icon }}</v-icon>
            </v-list-item-icon>
            <v-list-item-content>
              <v-list-item-title>{{ list.name }}</v-list-item-title>
            </v-list-item-content>
          </v-list-item>
        </v-list>
      </v-container>
    </v-navigation-drawer>
    <v-app-bar app color="primary" dark>
      <v-app-bar-nav-icon @click="drawer=!drawer"/>
      <v-toolbar-title>{{ $route.name }}</v-toolbar-title>
    </v-app-bar>

    <v-main>
      <router-view/>
    </v-main>
  </v-app>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator';

export default {
  data(){
    return{
        drawer: null,
        lists:[
          { name: 'Home', icon: 'mdi-home', link: '/home' },
          { name: 'Chat Log', icon: 'mdi-message-text', link: '/' },
          { name: 'Chart Line', icon: 'mdi-chart-line', link: '/chart'},
          { name: 'Admin Tool', icon: 'mdi-cog', link: '/admin'}
        ]
    }
  }
}
</script>
