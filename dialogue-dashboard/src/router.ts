import Vue from 'vue';
import Router from 'vue-router';
import HelloWorld from './components/HelloWorld.vue';
import ChatLog from './components/chatlog.vue';
import ChartLine from './components/chartline.vue';
import AdminTool from './components/adminTool.vue';

Vue.use(Router)

export default new Router({
  mode: 'history',
  base: process.env.BASE_URL,
  routes: [
    {
      path: '/home',
      name: 'Home',
      component: HelloWorld
    },
    {
      path: '/',
      name: 'Chat Log',
      component: ChatLog
    },
    {
      path: '/chart',
      name: 'Chart Line',
      component: ChartLine
    },
    {
      path: '/admin',
      name: 'Admin Tool',
      component: AdminTool
    }
  ]
});