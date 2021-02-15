import MainLayout from '@/layouts/MainLayout.vue';

export default {
  title: 'layouts/MainLayout',
  component: MainLayout,
};

export const Default = () => ({
  components: { MainLayout },
  template: `<main-layout/>`,
});
