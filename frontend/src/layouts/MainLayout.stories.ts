import MainLayout from "@/layouts/MainLayout.vue";

export default {
  title: 'layout/MainLayout',
  component: MainLayout,
};

export const Default = () => ({
  components: { MainLayout },
  template: `<MainLayout/>`
});